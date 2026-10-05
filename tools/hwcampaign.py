"""~1 h conducted receiver campaign (VSG60 -> pad -> ESP32-S3 pigtail). Every phase saves its plot and opens it.
    python tools/hwcampaign.py --atten 10 [--out DIR] [--port COM4] [--phases AGCBDEc] [--stab-secs 1200]
A  linearity / saturation: tone level vs VSG level for each gain, both IQ links -> picks the gains used below
G  tuning accuracy 2230-2780 MHz (10 MHz steps): received tone offset vs nominal
C  phase noise L(f), 2350/2480/2700 MHz, 62.5k (3 s blocks) + 250k (2 s blocks), per-block drift removal, VSG-off floor
B  frequency stability: one gapless capture, 50 ms frequency track, Allan deviation
D  residual FM (0.3-3 kHz) and phase-noise limited NBFM S/N vs level, 3 frequencies
E  NBFM SINAD with VSG60 FM (1 kHz tone, 3 kHz deviation) vs level, 3 frequencies
c  phase noise at 2350 MHz again (repeatability after the long run)
VSG is never above -50 dBm; levels at the DUT = VSG - atten."""
import argparse, json, os, sys, time, traceback
import numpy as np
from scipy import signal
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hwtest import VSG, Emu, iq_analyze, out_shift
import hwpn
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

FREQS = (2350e6, 2480e6, 2700e6)
P_LEVEL = -55.0                      # VSG level for P / B / G (bottom of the VSG60 standard range)
LEVELS = np.arange(-50, -85.1, -2.5)
OFF = {62500: 15e3, 250000: 40e3}


class Camp:
    def __init__(self, a):
        self.a = a; os.makedirs(a.out, exist_ok=True)
        self.log = open(os.path.join(a.out, "results.jsonl"), "a", encoding="utf-8")
        self.tmp = os.path.join(a.out, "cap.bin")
        self.v = VSG(); self.e = self.new_emu()
        self.g62 = 50; self.g250 = 30; self.gsweep = 50
        self.notes = []

    def rec(self, **d):
        d["t"] = round(time.time(), 3); self.log.write(json.dumps(d) + "\n"); self.log.flush()

    def say(self, s):
        print(time.strftime("%H:%M:%S"), s, flush=True); self.notes.append(time.strftime("%H:%M ") + s)

    def show(self, fig, name):
        p = os.path.join(self.a.out, name); fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
        try: os.startfile(p)
        except Exception: pass
        return p

    def new_emu(self):
        if self.a.dut == "esp32s3": return Emu(self.a.emu, self.a.port)
        import pn_ext                      # HackRF / RTL-SDR / BB60C with the same campaign code
        self.a.raw = os.path.join(self.a.out, "raw.bin"); return pn_ext.ExtEmu(self.a)

    def tune(self, f0, rate, gain, settle=0.4):
        g = int(gain) if self.a.dut == "esp32s3" else gain
        self.e.set(spec=0, rate=rate, freq=int(f0), gain=g, ppm="0.000", settle=settle)

    def cap(self, rate, secs, path=None):
        path = path or self.tmp
        ok, got, dt, _ = self.e.cap(int(rate * secs), path, timeout=secs * 3 + 10)
        return np.fromfile(path, np.complex64).astype(np.complex128) if got else None

    # ---------------- A: linearity / saturation ----------------
    def phase_a(self):
        f0 = 2350e6; gains = (10, 20, 30, 40, 50, 60, 70); levels = np.arange(-50, -85.1, -5)
        res = {}
        for rate in (62500, 250000):
            off = OFF[rate]
            for g in gains:
                self.tune(f0, rate, g)
                for p in levels:
                    self.v.set(f=f0 + off, p=float(p), on=True); time.sleep(0.1)
                    self.cap(rate, 0.4)
                    r = iq_analyze(self.tmp, rate, off, 6000)
                    fs = 2 ** (out_shift(g) + 7) / 32768.0 if rate == 250000 else 1.0
                    self.rec(test="A", rate=rate, gain=g, p=float(p), level=r["level"], peak=r["peak_abs"], fs=fs,
                             floor_bin=r["floor_bin"], snr=r["snr"], freq=r["freq"])
                    res[(rate, g, float(p))] = (r["level"], r["peak_abs"] / fs)
        self.v.set(on=False)

        def lin_ok(rate, g, p_hi):
            # 5 dB headroom: both 5 dB steps up to p_hi+5 must be +5 dB out (+-0.7), and the tone well above the
            # floor (the 8-bit link's full scale follows the gain, so linearity is the only reliable criterion)
            if p_hi + 5 > -50: p_hi = -55.0
            a, b, c = (res[(rate, g, q)] for q in (p_hi + 5, p_hi, p_hi - 5))
            return abs((a[0] - b[0]) - 5) < 0.7 and abs((b[0] - c[0]) - 5) < 0.7
        self.g62 = max([g for g in gains if lin_ok(62500, g, P_LEVEL)] or [gains[-1]])
        self.g250 = max([g for g in gains if lin_ok(250000, g, P_LEVEL)] or [30])
        self.gsweep = max([g for g in gains if lin_ok(62500, g, -55.0)] or [self.g62])
        self.rec(test="A_pick", g62=self.g62, g250=self.g250, gsweep=self.gsweep)
        self.say("A: linear gains -> 62.5k: %d, 250k (8-bit): %d, level sweep: %d" % (self.g62, self.g250, self.gsweep))
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
        for i, rate in enumerate((62500, 250000)):
            for g in gains:
                ax[i].plot(levels - self.a.atten, [res[(rate, g, float(p))][0] for p in levels], "o-", ms=3, label="gain %d" % g)
            ax[i].set_title("%s: tone level vs input" % ("62.5k (16-bit link)" if rate == 62500 else "250k (8-bit link)"))
            ax[i].set_xlabel("level at S3 input [dBm]"); ax[i].set_ylabel("tone [dBFS]"); ax[i].grid(alpha=0.3)
            ax[i].legend(fontsize=7)
        self.show(fig, "A_linearity.png")

    # ---------------- G: tuning accuracy across the band ----------------
    def phase_g(self):
        rate, off = 62500, OFF[62500]; out = []
        for f in np.arange(2230e6, 2780e6 + 1, 10e6):
            self.v.set(f=f + off, p=P_LEVEL, on=True)
            self.tune(f, rate, self.g62, settle=0.3)
            x = self.cap(rate, 1.0)
            ft = hwpn.refine_tone(x, rate, hwpn.find_tone(x, rate, off, 8000))
            err = ft - off; out.append((f, err))
            self.rec(test="G", f=float(f), err_hz=float(err), err_ppm=float(err / f * 1e6))
        self.v.set(on=False)
        f, e = np.array(out).T
        fig, ax = plt.subplots(figsize=(10, 4.5))
        ax.plot(f / 1e6, e, "o-", ms=3); ax.set_xlabel("tuned frequency [MHz]"); ax.set_ylabel("tone error [Hz] (ppm=0)")
        ax2 = ax.twinx(); ax2.plot(f / 1e6, e / f * 1e6, alpha=0); ax2.set_ylabel("[ppm]")
        ax.grid(alpha=0.3); ax.set_title("Tuning accuracy (VSG60 CW, ESP32-S3 62.5k IQ, PPM 0)")
        self.show(fig, "G_tuning.png")
        self.say("G: tone error %.0f .. %.0f Hz (%.2f .. %.2f ppm)" % (e.min(), e.max(), (e / f).min() * 1e6, (e / f).max() * 1e6))

    # ---------------- C: phase noise ----------------
    def floor(self, rate, nper, pc_db, secs):
        self.v.set(on=False); time.sleep(0.2)
        x = self.cap(rate, secs)
        f, S = signal.welch(x, rate, window="hann", nperseg=nper, noverlap=nper // 2, return_onesided=False,
                            detrend=False, scaling="density")
        f = np.fft.fftshift(f); S = np.fft.fftshift(S)
        return f, 10 * np.log10(S + 1e-30) - pc_db

    def phase_c(self, freqs=None, tag="C"):
        freqs = freqs or FREQS; curves = []
        for f0 in freqs:
            for rate, secs, blk, nper, g in ((62500, 60, 3.0, 2 ** 15, self.g62), (250000, 20, 2.0, 2 ** 15, self.g250)):
                off = OFF[rate]
                self.v.set(f=f0 + off, p=P_LEVEL, on=True)
                self.tune(f0, rate, g, settle=0.5)
                x = self.cap(rate, secs)
                fp, L, pc, nb, fts = hwpn.pn_curve_blocks(x, rate, nper, blk, off, 8000)
                ftm = float(np.mean(fts))
                keep = fp <= 0.85 * (rate / 2 - abs(ftm)); fp, L = fp[keep], L[keep]
                ff, Lf = self.floor(rate, nper, pc, 6 if rate == 62500 else 3)
                lf = np.interp(fp, ff[ff > 0], Lf[ff > 0])
                pts, flo = hwpn.pn_points(fp, L), hwpn.pn_points(fp, lf)
                rfm = hwpn.fm_from_pn(fp, L)
                np.savez(os.path.join(self.a.out, "%s_pn_%d_%d.npz" % (tag, int(f0 / 1e6), rate)), f=fp, L=L, floor=lf, pc=pc)
                self.rec(test=tag, f0=f0, rate=rate, gain=g, p_dut=P_LEVEL - self.a.atten, carrier_dbfs=pc, blocks=nb,
                         tone_hz=fts, L=pts, floor=flo, res_fm_pn_hz=rfm)
                curves.append((f0, rate, fp, L, lf))
                self.say("%s %.0f MHz %dk: L 100/1k/10k = %s / %s / %s dBc/Hz, floor %s, tone wander p-p %.0f Hz" % (
                    tag, f0 / 1e6, rate // 1000, *["%.0f" % pts[k] if pts[k] is not None else "-" for k in ("100", "1000", "10000")],
                    "%.0f" % flo["1000"] if flo["1000"] is not None else "-", np.ptp(fts)))
        fig, ax = plt.subplots(figsize=(10, 6))
        cols = {}
        for f0, rate, fp, L, lf in curves:
            m = (fp >= 5) & (fp < 3e3) if rate == 62500 else fp >= 3e3
            c = cols.setdefault(f0, None)
            ln, = ax.semilogx(fp[m], L[m], lw=0.8, color=c, label=None if c else "%.0f MHz" % (f0 / 1e6))
            cols[f0] = ln.get_color()
            ax.semilogx(fp[m], lf[m], lw=0.6, ls=":", color=cols[f0])
        k = sorted(hwpn.VSG60_PN_1G)
        ax.semilogx(k, [hwpn.VSG60_PN_1G[o] + 20 * np.log10(2.35) for o in k], "kD--", ms=4, label="VSG60 typ. (scaled to 2.35 GHz)")
        ax.set_xlim(5, 1.2e5); ax.set_ylim(-130, -30); ax.grid(True, which="both", alpha=0.3)
        ax.set_xlabel("offset [Hz]"); ax.set_ylabel("L(f) [dBc/Hz]"); ax.legend(fontsize=8)
        ax.set_title(self.a.dut + " conducted phase noise, %.0f dBm at input (dotted: VSG off floor)" % (P_LEVEL - self.a.atten))
        self.show(fig, "%s_phase_noise.png" % tag)

    # ---------------- B: frequency stability ----------------
    def phase_b(self, secs):
        if self.a.dut != "esp32s3": return self.phase_b_blocks(secs)
        f0, rate, off = 2350e6, 62500, OFF[62500]
        path = os.path.join(self.a.out, "stab.cf32")
        self.v.set(f=f0 + off, p=P_LEVEL, on=True)
        self.tune(f0, rate, self.g62, settle=0.5)
        s0 = self.e.stats()
        ok, got, dt, _ = self.e.cap(int(rate * secs), path, timeout=secs + 120)
        s1 = self.e.stats()
        self.v.set(on=False)
        x = np.memmap(path, np.complex64, "r")
        seg = rate // 20; nfft = 1 << 14; w = np.hanning(seg); f = np.fft.fftfreq(nfft, 1 / rate)
        win = np.abs(f - off) < 8000
        nseg = len(x) // seg; fr = np.empty(nseg); lv = np.empty(nseg)
        for c0 in range(0, nseg, 2000):
            c1 = min(nseg, c0 + 2000)
            X = np.asarray(x[c0 * seg:c1 * seg]).reshape(c1 - c0, seg) * w
            S = np.abs(np.fft.fft(X, nfft, axis=1)) ** 2
            S[:, ~win] = 0
            j = np.argmax(S, axis=1); r = np.arange(c1 - c0)
            a_, b_, c_ = (np.log(S[r, (j + d) % nfft] + 1e-30) for d in (-1, 0, 1))
            den = a_ - 2 * b_ + c_; d = np.where(den != 0, 0.5 * (a_ - c_) / np.where(den != 0, den, 1), 0)
            fr[c0:c1] = f[j] + d * rate / nfft - off; lv[c0:c1] = 10 * np.log10(S[r, j] / np.sum(w) ** 2 + 1e-30)
        del x
        try: os.remove(path)
        except OSError: pass
        tau0 = seg / rate; y = fr / f0; t = np.arange(nseg) * tau0
        ph = np.concatenate([[0], np.cumsum(y) * tau0])          # time error [s]
        taus, adev = [], []; m = 1
        while 3 * m < len(ph):
            d = ph[2 * m:] - 2 * ph[m:-m] + ph[:-2 * m]
            taus.append(m * tau0); adev.append(float(np.sqrt(np.mean(d ** 2) / (2 * (m * tau0) ** 2)))); m = int(np.ceil(m * 1.5))
        self.rec(test="B", secs=secs, f0=f0, mean_hz=float(fr.mean()), pp_hz=float(np.ptp(fr)), std_hz=float(fr.std()),
                 tau=taus, adev=adev, level_pp=float(np.ptp(lv)), gaps=s1["gaps"] - s0["gaps"], lost=s1["lost"] - s0["lost"])
        fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
        ax[0].plot(t / 60, fr, lw=0.5); ax[0].set_xlabel("time [min]"); ax[0].set_ylabel("tone offset [Hz] @ 2350 MHz")
        ax[0].grid(alpha=0.3); ax[0].set_title("frequency vs time (50 ms resolution)")
        ax[1].loglog(taus, adev, "o-", ms=3); ax[1].set_xlabel("tau [s]"); ax[1].set_ylabel("Allan deviation")
        ax[1].grid(True, which="both", alpha=0.3); ax[1].set_title("ADEV (S3 crystal vs VSG60 reference)")
        self.show(fig, "B_stability.png")
        self.say("B: %.0f s, offset %.0f Hz, p-p %.0f Hz, ADEV(1 s) %.1e, ADEV(100 s) %.1e, gaps %d" % (
            secs, fr.mean(), np.ptp(fr), np.interp(1, taus, adev), np.interp(100, taus, adev), s1["gaps"] - s0["gaps"]))


    def phase_b_blocks(self, secs, blk=10.0):
        """External receivers: 10 s captures back to back for `secs` (short gaps between them), 50 ms frequency
        track inside each block; ADEV from the in-block data up to 2 s, block means show the slow drift."""
        f0, rate, off = 2350e6, 62500, OFF[62500]
        self.v.set(f=f0 + off, p=P_LEVEL, on=True); self.tune(f0, rate, self.g62, settle=0.3)
        seg = rate // 20; t0 = time.time(); T, F, adev_acc = [], [], {}
        while time.time() - t0 < secs:
            x = self.cap(rate, blk); tb = time.time() - t0
            fr = []
            for i in range(len(x) // seg):
                b = x[i * seg:(i + 1) * seg]; ft = hwpn.find_tone(b, rate, off, 8000)
                fr.append(hwpn.refine_tone(b, rate, ft) - off)
            fr = np.array(fr); T.extend(tb + np.arange(len(fr)) * seg / rate); F.extend(fr)
            y = fr / f0; ph = np.concatenate([[0], np.cumsum(y) * seg / rate])
            for m in (1, 2, 4, 8, 20, 40):
                if 3 * m < len(ph):
                    d = ph[2 * m:] - 2 * ph[m:-m] + ph[:-2 * m]
                    adev_acc.setdefault(m, []).append(np.mean(d ** 2) / (2 * (m * seg / rate) ** 2))
        self.v.set(on=False)
        T, F = np.array(T), np.array(F)
        taus = [m * seg / rate for m in sorted(adev_acc)]; adev = [float(np.sqrt(np.mean(adev_acc[m]))) for m in sorted(adev_acc)]
        self.rec(test="B", secs=secs, f0=f0, mean_hz=float(F.mean()), pp_hz=float(np.ptp(F)), tau=taus, adev=adev, blocks=True)
        fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
        ax[0].plot(T / 60, F, ",", ms=1); ax[0].set_xlabel("time [min]"); ax[0].set_ylabel("tone offset [Hz] @ 2350 MHz"); ax[0].grid(alpha=0.3)
        ax[0].set_title("%s frequency vs time (10 s blocks, 50 ms resolution)" % self.a.dut)
        ax[1].loglog(taus, adev, "o-", ms=3); ax[1].set_xlabel("tau [s]"); ax[1].set_ylabel("Allan deviation"); ax[1].grid(True, which="both", alpha=0.3)
        self.show(fig, "B_stability.png")
        self.say("B (blocks): %.0f s, offset %.0f Hz, p-p %.0f Hz, ADEV(1 s) %.1e" % (secs, F.mean(), np.ptp(F), np.interp(1, taus, adev)))

    # ---------------- D / E: residual FM, NBFM SINAD vs level ----------------
    @staticmethod
    def fm_carrier(x, rate, guess):
        """Carrier of a sine-FM signal = power-weighted centre of its spectrum (independent of the crystal
        offset; the first version demodulated around the nominal offset and clipped the sidebands when the
        S3 sat ~4 kHz off -> erratic SINAD)."""
        f, P = signal.periodogram(x, rate, window="hann", return_onesided=False, detrend=False)
        fc = guess
        for half in (14e3, 9e3, 8e3):
            m = np.abs(f - fc) < half
            fc = float(np.sum(f[m] * P[m]) / np.sum(P[m]))
        return fc

    def fm_on(self, on):
        if on:
            for c in (":SOURce:FM:SHAPe SINE", ":SOURce:FM:FREQuency 1000", ":SOURce:FM:DEViation 3000",
                      ":SOURce:FM:STATe ON", ":OUTPut:MODulation:STATe ON"):
                self.v.w(c)
        else:
            self.v.w(":SOURce:FM:STATe OFF"); self.v.w(":OUTPut:MODulation:STATe OFF")
        self.v.set(p=-50)

    def sweep(self, tag, fm):
        rate, off = 62500, OFF[62500]; rows = []
        if fm: self.fm_on(True)
        try:
            for f0 in FREQS:
                self.tune(f0, rate, self.gsweep, settle=0.5)
                if fm:   # check the FM really reaches the output (block median: robust to the VSG's dropouts)
                    okfm = False
                    for attempt in range(4):
                        self.v.set(f=f0 + off, p=-50, on=True); time.sleep(0.3)
                        x = self.cap(rate, 2.0)
                        fi = hwpn.fm_demod(x, rate, self.fm_carrier(x, rate, off))
                        dv = hwpn.sinad_blocks(fi, rate, x=x, tone=1e3)["dev_peak_hz"]
                        if 2500 < dv < 3500: okfm = True; break
                        self.say("E %.0f MHz: FM check %d: deviation %.0f Hz, retrying" % (f0 / 1e6, attempt, dv))
                    if not okfm:
                        self.say("E %.0f MHz: FM not reaching the output -> this frequency skipped" % (f0 / 1e6)); continue
                for p in LEVELS:
                    self.v.set(f=f0 + off, p=float(p), on=True); time.sleep(0.15)
                    x = self.cap(rate, 4.0)
                    if fm:
                        fc = self.fm_carrier(x, rate, off)
                        fi = hwpn.fm_demod(x, rate, fc)
                        m = hwpn.audio_metrics(fi, rate, tone=1e3)
                        m["sinad_4s_db"] = m.pop("sinad_db")
                        m.update(hwpn.sinad_blocks(fi, rate, x=x, tone=1e3)); y = m["sinad_db"]
                        m["fc_hz"] = fc
                    else:
                        ft = hwpn.refine_tone(x, rate, hwpn.find_tone(x, rate, off, 8000))
                        m = hwpn.audio_metrics(hwpn.fm_demod(x, rate, ft), rate)
                        m["snr_db"] = hwpn.snr_for_dev(m["audio_rms_hz"]); y = m["snr_db"]["3000"]
                    pc = 10 * np.log10(np.mean(np.abs(x) ** 2) + 1e-30)
                    self.rec(test=tag, f0=f0, p=float(p), p_dut=float(p) - self.a.atten, gain=self.gsweep, carrier_dbfs=pc, **m)
                    rows.append((f0, float(p) - self.a.atten, y, m["audio_rms_hz"] if not fm else m["dev_peak_hz"]))
                self.say("%s %.0f MHz: %s" % (tag, f0 / 1e6, " ".join("%.0f:%.1f" % (r[1], r[2]) for r in rows if r[0] == f0)))
        finally:
            if fm: self.fm_on(False)
            self.v.set(on=False)
        fig, ax = plt.subplots(figsize=(8, 5))
        for f0 in FREQS:
            s = [r for r in rows if r[0] == f0]
            ax.plot([r[1] for r in s], [r[2] for r in s], "o-", ms=3, label="%.0f MHz" % (f0 / 1e6))
        ax.axhline(12, color="k", lw=0.6, ls="--")
        ax.set_xlabel("level at S3 input [dBm]"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
        ax.set_ylabel("SINAD [dB] (1 kHz, 3 kHz dev)" if fm else "S/N limit from residual FM, 3 kHz dev [dB]")
        ax.set_title("%s NBFM %s vs level, gain %s" % (self.a.dut, "SINAD" if fm else "residual-FM S/N", self.gsweep))
        self.show(fig, "%s_nbfm.png" % tag)

    def guarded(self, name, fn, *args):
        t = time.time(); self.say("--- phase %s start" % name)
        try:
            fn(*args); self.rec(test="phase", name=name, ok=True, secs=time.time() - t)
        except Exception as e:
            self.rec(test="phase", name=name, ok=False, error=repr(e), tb=traceback.format_exc()[-1500:])
            self.say("!! %s failed: %r" % (name, e))
            try:
                self.e.quit()
            except Exception: pass
            time.sleep(2); self.e = self.new_emu()
            self.e.cmd("set spec=0 rate=62500 freq=2350000000 gain=30"); self.e.start(); self.e.cmd("sync 8")

    def close(self):
        try: self.fm_on(False)
        except Exception: pass
        try: self.v.set(on=False)
        except Exception: pass
        try: self.e.quit()
        except Exception: pass
        with open(os.path.join(self.a.out, "SUMMARY.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(self.notes) + "\n")


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--emu", default=os.path.join(here, "..", "build", "Release", "esp_sdr_emu.exe"))
    ap.add_argument("--out", default=time.strftime("campaign_%Y%m%d_%H%M"))
    ap.add_argument("--atten", type=float, required=True)
    ap.add_argument("--phases", default="AGCBDEc")
    ap.add_argument("--stab-secs", type=float, default=1200)
    ap.add_argument("--gains", default=None, help="g62,g250,gsweep (skip A); hackrf: lna:vga for all, e.g. 16:20")
    ap.add_argument("--freqs", default=None, help="comma-separated Hz, default 2350e6,2480e6,2700e6")
    ap.add_argument("--dut", default="esp32s3", choices=["esp32s3", "hackrf", "rtlsdr", "bb60c"])
    ap.add_argument("--levels", default=None, help="VSG sweep lo,hi,step in dBm, e.g. -120,-80,2")
    a = ap.parse_args()
    c = Camp(a)
    if a.gains and a.dut == "hackrf": c.g62 = c.g250 = c.gsweep = tuple(int(v) for v in a.gains.split(":"))
    elif a.gains and a.dut in ("rtlsdr", "bb60c"): c.g62 = c.g250 = c.gsweep = float(a.gains)
    elif a.gains: c.g62, c.g250, c.gsweep = (int(v) for v in a.gains.split(","))
    global FREQS
    if a.freqs: FREQS = tuple(float(v) for v in a.freqs.split(","))
    if a.levels:
        global LEVELS
        lo, hi, st = (float(v) for v in a.levels.split(","))
        LEVELS = np.arange(hi, lo - 0.01, -abs(st))
    try:
        c.e.cmd("set spec=0 rate=62500 freq=2350000000 gain=30"); c.e.start(); c.e.cmd("sync 8")
        c.rec(test="start", args=vars(a))
        steps = {"A": (c.phase_a,), "G": (c.phase_g,), "C": (c.phase_c,), "B": (c.phase_b, a.stab_secs),
                 "D": (c.sweep, "D", False), "E": (c.sweep, "E", True), "c": (c.phase_c, (2350e6,), "c")}
        for k in a.phases:
            c.guarded(k, *steps[k])
        c.rec(test="end"); c.say("campaign done")
    finally:
        c.close()


if __name__ == "__main__":
    main()
