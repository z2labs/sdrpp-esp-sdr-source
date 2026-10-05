"""Close-in phase noise and NBFM performance of the ESP32-S3 receive chain (VSG60 CW / FM as the source):
    python tools/hwpn.py [--tests PRF] [--f0 2350e6,2480e6,2700e6] [--out DIR] [--port COM4]
    python tools/hwpn.py --selftest          # checks the L(f) / residual-FM maths on synthetic data, no hardware
P  L(f) in dBc/Hz, 10 Hz .. 100 kHz offset, IQ 62.5k (30 s) and 250k (8 s), plus the VSG-off floor at the same gain.
   This is VSG60 + S3 together: VSG60 typ. -89/-114/-125/-127 dBc/Hz @ 100 Hz/1k/10k/100k at 1 GHz
   (+7.4 dB at 2.35 GHz), so above ~1 kHz offset the result is the S3.
R  residual FM (rms Hz, 300-3000 Hz audio band, no de-emphasis) of the CW vs VSG level, and the phase-noise
   limited NBFM S/N for a 1 kHz tone at 2.5 / 3 / 5 kHz peak deviation.
F  real NBFM SINAD: VSG60 FM, 1 kHz sine, 3 kHz deviation, vs VSG level (radiated setup: levels are relative).
Results: results.jsonl, pn_*.npz and PNGs in the output folder."""
import argparse, os, sys, time
import numpy as np
from scipy import signal
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hwtest
from hwtest import iq_analyze

OFFS = (10, 30, 100, 300, 1e3, 3e3, 10e3, 30e3, 100e3)
AUDIO = (300.0, 3000.0)
VSG60_PN_1G = {100: -89, 1e3: -114, 10e3: -125, 100e3: -127}


# ---------------- maths (also used by --selftest) ----------------
def find_tone(x, fs, f_exp, search):
    f, P = signal.periodogram(x, fs, window="hann", return_onesided=False, detrend=False)
    m = np.abs(f - f_exp) <= search
    return float(f[m][np.argmax(P[m])]) if np.any(m) else float(f[np.argmax(P)])


def refine_tone(x, fs, f1):
    """Mix to ~0, then the mean phase slope gives the tone frequency to sub-Hz."""
    z = x * np.exp(-2j * np.pi * f1 * np.arange(len(x)) / fs)
    k = max(1, int(fs / 2000))
    zz = z[: len(z) // k * k].reshape(-1, k).mean(axis=1)
    ph = np.unwrap(np.angle(zz))
    return f1 + np.polyfit(np.arange(len(ph)) * k / fs, ph, 1)[0] / (2 * np.pi)


def pn_curve(x, fs, f_tone, nper):
    """Two-sided PSD of the carrier-centred signal, normalised to the carrier power -> L(f) [dBc/Hz] for f > 0
    (mean of upper and lower sideband) and the carrier power (dBFS)."""
    t = np.arange(len(x)) / fs
    z = x * np.exp(-2j * np.pi * f_tone * t)
    # remove the slow crystal drift (cubic phase over the whole capture, ~ < 0.1 Hz), otherwise a few-Hz wander
    # over 30 s smears the carrier across the close-in offsets
    k = max(1, int(fs / 2000))
    zz = z[: len(z) // k * k].reshape(-1, k).mean(axis=1)
    tt = (np.arange(len(zz)) + 0.5) * k / fs
    q = np.polyfit(tt, np.unwrap(np.angle(zz)), 3)
    z = z * np.exp(-1j * np.polyval(q, t))
    f, S = signal.welch(z, fs, window="hann", nperseg=nper, noverlap=nper // 2, return_onesided=False,
                        detrend=False, scaling="density")
    f = np.fft.fftshift(f); S = np.fft.fftshift(S)
    df = fs / nper
    pc = S[np.abs(f) <= 3 * df].sum() * df              # Hann main lobe holds the carrier
    pos = f > 3 * df
    fp = f[pos]
    neg = np.interp(fp, -f[::-1], S[::-1])               # S(-f)
    L = 10 * np.log10((S[pos] + neg) / 2 / pc + 1e-30)
    return fp, L, 10 * np.log10(pc + 1e-30)


def pn_curve_blocks(x, fs, nper, blk_secs, f_exp, search=3000):
    """L(f) averaged over short blocks, each with its own tone estimate and drift removal, so slow thermal
    wander of the crystal (tens of Hz over seconds) does not leak into the close-in offsets.
    Returns f, L, mean carrier power, block count, list of per-block tone frequencies."""
    n = int(blk_secs * fs); acc = None; pcs = []; fts = []
    for i in range(len(x) // n):
        b = x[i * n:(i + 1) * n]
        ft = refine_tone(b, fs, find_tone(b, fs, f_exp, search))
        fp, L, pc = pn_curve(b, fs, ft, nper)
        lin = 10 ** (L / 10); acc = lin if acc is None else acc + lin; pcs.append(pc); fts.append(ft)
    return fp, 10 * np.log10(acc / max(1, len(pcs))), float(np.mean(pcs)), len(pcs), fts


def pn_points(fp, L, offs=OFFS, frac=0.15):
    """Median L in +-15 % around each offset (robust against spurs); None outside the measured range."""
    out = {}
    for o in offs:
        m = (fp >= o * (1 - frac)) & (fp <= o * (1 + frac))
        out[str(int(o))] = float(np.median(L[m])) if m.sum() >= 3 else None
    return out


def fm_from_pn(fp, L, band=AUDIO):
    """rms residual FM [Hz] = sqrt(int S_phi(f) f^2 df), S_phi one-sided = 2 L(f)."""
    m = (fp >= band[0]) & (fp <= band[1])
    return float(np.sqrt(np.trapezoid(2 * 10 ** (L[m] / 10) * fp[m] ** 2, fp[m]))) if m.sum() > 2 else None


def fm_demod(x, fs, f_c, if_half=7.5e3):
    """Ideal NBFM receiver: carrier to 0 Hz, brick-wall IF +-if_half (12.5 kHz-ish channel), discriminator -> Hz."""
    z = x * np.exp(-2j * np.pi * f_c * np.arange(len(x)) / fs)
    Z = np.fft.fft(z); fz = np.fft.fftfreq(len(z), 1 / fs)
    Z[np.abs(fz) > if_half] = 0
    z = np.fft.ifft(Z)
    return np.angle(z[1:] * np.conj(z[:-1])) * fs / (2 * np.pi)


def audio_metrics(fi, fs, tone=None, band=AUDIO):
    """rms of the discriminator output in the audio band; with a test tone also SINAD and its peak deviation."""
    fi = fi[int(0.01 * fs):]                              # drop the filter edge
    f, P = signal.periodogram(fi - np.mean(fi), fs, window="hann", detrend="linear", scaling="density")
    df = f[1] - f[0]
    m = (f >= band[0]) & (f <= band[1])
    tot = P[m].sum() * df
    r = dict(audio_rms_hz=float(np.sqrt(tot)), dc_hz=float(np.mean(fi)))
    if tone:
        t = m & (np.abs(f - tone) <= 6)
        fund = P[t].sum() * df
        r.update(sinad_db=float(10 * np.log10(tot / max(tot - fund, 1e-30))), dev_peak_hz=float(np.sqrt(2 * fund)),
                 noise_dist_rms_hz=float(np.sqrt(max(tot - fund, 0))))
    return r


def sinad_blocks(fi, fs, x=None, tone=1e3, blk=0.25, band=AUDIO, dip_db=1.0):
    """SINAD per 0.25 s block (like a SINAD meter's notch, +-12 Hz), median over blocks. Blocks where the
    carrier amplitude dips by > dip_db (5 ms averages vs the capture median) are counted separately: the
    VSG60 streams FM from the PC and drops out now and then (amplitude dips with +-30 kHz clicks) - source
    glitches, not receiver noise. Discriminator clicks without an amplitude dip (threshold effect at low
    level) stay in, they are the receiver's."""
    n = int(blk * fs); s_all = []; s_clean = []; devs = []; clicks = 0
    if x is not None:
        k = max(1, int(0.005 * fs)); a = np.abs(x[: len(x) // k * k]).reshape(-1, k).mean(axis=1)
        a_db = 20 * np.log10(a / np.median(a) + 1e-12)
    for i in range(len(fi) // n):
        b = fi[i * n:(i + 1) * n]
        f, P = signal.periodogram(b - b.mean(), fs, window="hann", detrend="linear", scaling="density")
        df = f[1] - f[0]; m = (f >= band[0]) & (f <= band[1]); t = m & (np.abs(f - tone) <= 12)
        tot, fund = P[m].sum() * df, P[t].sum() * df
        s = 10 * np.log10(tot / max(tot - fund, 1e-30)); s_all.append(s)
        dip = x is not None and np.min(a_db[i * n // k:(i + 1) * n // k]) < -dip_db
        if dip: clicks += 1
        else: s_clean.append(s); devs.append(np.sqrt(2 * fund))
    nb = len(s_all)
    return dict(sinad_db=float(np.median(s_clean)) if s_clean else float("nan"),
                sinad_all_db=float(np.median(s_all)) if s_all else float("nan"),
                dev_peak_hz=float(np.median(devs)) if devs else 0.0, blocks=nb, glitch_blocks=clicks)


def snr_for_dev(res_rms, devs=(2500, 3000, 5000)):
    return {str(d): float(20 * np.log10(d / np.sqrt(2) / max(res_rms, 1e-9))) for d in devs}


def selftest():
    """Synthetic CW with known phase noise: white FM (L = -80 dBc/Hz @ 1 kHz, -20 dB/dec) + white PM (-108 dBc/Hz)."""
    fs, secs, off = 62500, 30, 10e3
    n = fs * secs; rng = np.random.default_rng(1)
    sw = np.sqrt(1e-8 * (2 * np.pi * 1e3) ** 2 / fs)      # random-walk phase step -> L(1k) = -80 dBc/Hz
    sp = np.sqrt(10 ** (-108 / 10) * fs)                    # white phase -> L = -108 dBc/Hz
    ph = np.cumsum(rng.normal(0, sw, n)) + rng.normal(0, sp, n)
    x = 0.5 * np.exp(1j * (2 * np.pi * off * np.arange(n) / fs + ph))
    # 62.5k is the 16-bit link. (The 8-bit 250k link: quantisation floor ~ -98 dBc/Hz for a carrier 6 dB below FS.)
    x = (np.round(x.real * 32767) + 1j * np.round(x.imag * 32767)) / 32767
    ft = refine_tone(x, fs, find_tone(x, fs, off, 2000))
    fp, L, pc = pn_curve(x, fs, ft, 2 ** 16)
    pts = pn_points(fp, L)
    exp = {o: 10 * np.log10(10 ** ((-80 - 20 * np.log10(o / 1e3)) / 10) + 10 ** (-108 / 10)) for o in (100, 1e3, 10e3)}
    ok = True
    for o, e in exp.items():
        got = pts[str(int(o))]; ok &= abs(got - e) < 1.5
        print("L(%6.0f Hz): measured %7.1f  expected %7.1f dBc/Hz" % (o, got, e))
    sf = 2 * sw ** 2 * fs / (4 * np.pi ** 2)               # white FM, one-sided [Hz^2/Hz]
    spm = 2 * 10 ** (-108 / 10)                             # white PM, one-sided S_phi
    e_fm = np.sqrt(sf * (AUDIO[1] - AUDIO[0]) + spm * (AUDIO[1] ** 3 - AUDIO[0] ** 3) / 3)
    r = audio_metrics(fm_demod(x, fs, ft), fs)
    print("residual FM: from L(f) %.2f Hz, discriminator %.2f Hz, expected %.2f Hz rms" % (
        fm_from_pn(fp, L), r["audio_rms_hz"], e_fm))
    ok &= abs(fm_from_pn(fp, L) / e_fm - 1) < 0.1 and abs(r["audio_rms_hz"] / e_fm - 1) < 0.1
    # FM tone: 1 kHz, 3 kHz deviation, SINAD must be ~ phase-noise limited S/N
    t = np.arange(n) / fs
    y = 0.5 * np.exp(1j * (2 * np.pi * off * t + 3.0 * np.sin(2 * np.pi * 1e3 * t) + ph))
    a = audio_metrics(fm_demod(y, fs, off), fs, tone=1e3)
    print("FM 3 kHz dev: measured dev %.0f Hz, SINAD %.1f dB, expected S/N %.1f dB" % (
        a["dev_peak_hz"], a["sinad_db"], snr_for_dev(e_fm)["3000"]))
    ok &= abs(a["dev_peak_hz"] - 3000) < 60 and abs(a["sinad_db"] - snr_for_dev(e_fm)["3000"]) < 1.0
    print("SELFTEST", "PASS" if ok else "FAIL")
    return ok


# ---------------- hardware ----------------
class NoVSG:
    """--no-vsg: lets the S3 side run (floor captures, pipeline check) while the generator is away."""
    def __init__(self, *a, **k): pass
    def w(self, c): pass
    def q(self, c): return ""
    def set(self, **k): pass


def load_cf32(path):
    return np.fromfile(path, np.complex64).astype(np.complex128)


class PN:
    LEVELS = np.arange(-50, -85.1, -2.5)   # VSG60: -55 dBm standard, -85 dBm extended range; lower via a pad (--atten)

    def __init__(self, a):
        from hwlong import Long
        self.r = Long(a); self.a = a

    def cap(self, rate, secs):
        ok, got, dt, _ = self.r.emu.cap(int(rate * secs), self.r.tmp, timeout=secs * 3 + 5)
        return load_cf32(self.r.tmp) if got else None

    def floor_rel(self, rate, nper, pc_db):
        """VSG off, same gain: noise floor relative to the carrier power measured before [dBc/Hz]."""
        self.r.vsg.set(on=False); time.sleep(0.2)
        x = self.cap(rate, max(2.0, 4 * nper / rate))
        f, S = signal.welch(x, rate, window="hann", nperseg=nper, noverlap=nper // 2, return_onesided=False,
                            detrend=False, scaling="density")
        f = np.fft.fftshift(f); S = np.fft.fftshift(S)
        return f, 10 * np.log10(S + 1e-30) - pc_db

    def test_p(self, f0):
        out = {}
        for rate, secs, off, nper in ((62500, 30, 10e3, 2 ** 16), (250000, 8, 25e3, 2 ** 15)):
            if self.a.quick: secs = min(secs, 4)
            self.r.vsg.set(f=f0 + off, p=-50, on=True)
            self.r.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.r.gain, settle=0.5)
            x = self.cap(rate, secs)
            if x is None: self.r.say("P %d: no data" % rate); continue
            ft = refine_tone(x, rate, find_tone(x, rate, off, 3000))
            fp, L, pc = pn_curve(x, rate, ft, nper)
            keep = fp <= 0.85 * (rate / 2 - abs(ft))           # upper sideband must stay inside the passband
            fp, L = fp[keep], L[keep]
            fi = fm_demod(x, rate, ft); am = audio_metrics(fi, rate)
            ff, Lf = self.floor_rel(rate, nper, pc)
            pos = ff > 0
            lf = np.interp(fp, ff[pos], Lf[pos])
            name = "pn_%d_%d" % (int(f0 / 1e6), rate)
            np.savez(os.path.join(self.a.out, name + ".npz"), f=fp, L=L, floor=lf, pc_dbfs=pc, f_tone=ft)
            pts = pn_points(fp, L); flo = pn_points(fp, lf)
            self.r.rec(test="P", f0=f0, rate=rate, secs=secs, f_tone=ft, carrier_dbfs=pc, L=pts, floor=flo,
                       res_fm_pn_hz=fm_from_pn(fp, L), res_fm_disc_hz=am["audio_rms_hz"],
                       snr_db=snr_for_dev(am["audio_rms_hz"]), gain=self.r.gain)
            out[rate] = pts
            self.r.say("P %.0f MHz %d: " % (f0 / 1e6, rate) + " ".join(
                "%s:%s" % (k, "%.0f" % v if v is not None else "-") for k, v in pts.items()) +
                "  resFM %.1f Hz" % am["audio_rms_hz"])
        self.r.vsg.set(on=False)
        return out

    def level_sweep(self, f0, test, fm):
        rate, off = 62500, 10e3
        self.r.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.r.gain, settle=0.5)
        for p in self.LEVELS:
            self.r.vsg.set(f=f0 + off, p=float(p), on=True); time.sleep(0.15)
            x = self.cap(rate, 2.0 if self.a.quick else 4.0)
            if x is None: continue
            if fm:
                fc = off + np.mean(fm_demod(x, rate, off))          # carrier = mean of the discriminator
                m = audio_metrics(fm_demod(x, rate, fc), rate, tone=1e3)
                pc = 10 * np.log10(np.mean(np.abs(x) ** 2) + 1e-30)
            else:
                ft = refine_tone(x, rate, find_tone(x, rate, off, 3000))
                fp, L, pc = pn_curve(x, rate, ft, 2 ** 14)
                m = audio_metrics(fm_demod(x, rate, ft), rate)
                m["snr_db"] = snr_for_dev(m["audio_rms_hz"])
            self.r.rec(test=test, f0=f0, p=float(p), p_dut=float(p) - self.a.atten, atten=self.a.atten,
                       carrier_dbfs=pc, gain=self.r.gain, **m)
            self.r.say("%s %.0f MHz %+.1f dBm at DUT: carrier %.1f dBFS  %s" % (test, f0 / 1e6, p - self.a.atten, pc, (
                "SINAD %.1f dB dev %.0f Hz" % (m["sinad_db"], m["dev_peak_hz"]) if fm else
                "resFM %.1f Hz  S/N(3k) %.1f dB" % (m["audio_rms_hz"], m["snr_db"]["3000"]))))
        self.r.vsg.set(p=-50, on=False)

    def test_r(self, f0):
        self.level_sweep(f0, "R", fm=False)

    def fm_on(self, on, dev=3000):
        v = self.r.vsg
        if on:
            v.w(":SOURce:FM:SHAPe SINE"); v.w(":SOURce:FM:FREQuency 1000"); v.w(":SOURce:FM:DEViation %d" % dev)
            v.w(":SOURce:FM:STATe ON"); v.w(":OUTPut:MODulation:STATe ON")
        else:
            v.w(":SOURce:FM:STATe OFF"); v.w(":OUTPut:MODulation:STATe OFF")
        v.set(p=-50)

    def test_f(self, f0):
        self.fm_on(True)
        try:
            self.r.vsg.set(f=f0 + 10e3, p=-50, on=True)
            self.r.emu.set(spec=0, rate=62500, freq=int(f0), gain=self.r.gain, settle=0.5)
            x = self.cap(62500, 2.0)
            m = audio_metrics(fm_demod(x, 62500, 10e3 + np.mean(fm_demod(x, 62500, 10e3))), 62500, tone=1e3)
            if not 2000 < m["dev_peak_hz"] < 4000:
                self.r.rec(test="F", f0=f0, error="FM not reaching the output", **m)
                self.r.say("F: FM check failed, measured deviation %.0f Hz -> skipped" % m["dev_peak_hz"]); return
            self.level_sweep(f0, "F", fm=True)
        finally:
            self.fm_on(False)


# ---------------- plots ----------------
def plots(out):
    import json, glob
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    recs = [json.loads(l) for l in open(os.path.join(out, "results.jsonl"), encoding="utf-8")]
    files = sorted(glob.glob(os.path.join(out, "pn_*.npz")))
    if files:
        fig, ax = plt.subplots(figsize=(9, 5.5))
        for fn in files:
            d = np.load(fn); tag = os.path.basename(fn)[3:-4].replace("_", " MHz, ") + " S/s"
            lo = 10 if "62500" in fn else 3e3                       # 62.5k for the close-in part, 250k beyond
            m = d["f"] >= lo
            ax.semilogx(d["f"][m], d["L"][m], lw=0.8, label=tag)
            ax.semilogx(d["f"][m], d["floor"][m], lw=0.6, ls=":", color=ax.lines[-1].get_color())
        f0 = np.mean([r["f0"] for r in recs if r.get("test") == "P"])
        k = sorted(VSG60_PN_1G)
        ax.semilogx(k, [VSG60_PN_1G[o] + 20 * np.log10(f0 / 1e9) for o in k], "kD--", ms=5,
                    label="VSG60 typ. spec, scaled to %.0f MHz" % (f0 / 1e6))
        ax.set_xlim(10, 1.2e5); ax.set_ylim(-130, -30); ax.grid(True, which="both", alpha=0.3)
        ax.set_xlabel("offset from carrier [Hz]"); ax.set_ylabel("L(f) [dBc/Hz]")
        st = [r for r in recs if r.get("test") == "start"]
        dut = st[0].get("dut", "ESP32-S3") if st else "ESP32-S3"
        ax.set_title("%s receive chain phase noise (VSG60 CW, dotted: VSG off floor)" % dut)
        ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(os.path.join(out, "phase_noise.png"), dpi=130)
    for test, key, ylab in (("R", None, "phase-noise limited S/N, 3 kHz dev [dB]"), ("F", "sinad_db", "SINAD [dB]")):
        rr = [r for r in recs if r.get("test") == test and "p" in r]
        if not rr: continue
        fig, ax = plt.subplots(figsize=(7, 4.5))
        for f0 in sorted({r["f0"] for r in rr}):
            s = [r for r in rr if r["f0"] == f0]
            y = [r[key] if key else r["snr_db"]["3000"] for r in s]
            ax.plot([r.get("p_dut", r["p"]) for r in s], y, "o-", ms=3, label="%.0f MHz" % (f0 / 1e6))
        ax.axhline(12, color="k", lw=0.6, ls="--")
        ax.set_xlabel("level at the DUT input [dBm] (VSG60 minus pad)"); ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8); fig.tight_layout()
        fig.savefig(os.path.join(out, "nbfm_%s.png" % test), dpi=130)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--emu", default=os.path.join(here, "..", "build", "Release", "esp_sdr_emu.exe"))
    ap.add_argument("--out", default=time.strftime("hwpn_%Y%m%d_%H%M"))
    ap.add_argument("--freqs", default="2350e6,2480e6,2700e6")
    ap.add_argument("--gain", type=int, default=None)
    ap.add_argument("--tests", default="PRF")
    ap.add_argument("--quick", action="store_true", help="short captures (pipeline check)")
    ap.add_argument("--no-vsg", action="store_true", help="generator not connected: only the S3 side runs")
    ap.add_argument("--atten", type=float, default=0.0, help="pad between VSG and DUT [dB], for the level axis")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--plots-only", action="store_true")
    a = ap.parse_args()
    if a.selftest: sys.exit(0 if selftest() else 1)
    if a.plots_only: plots(a.out); return
    freqs = [float(v) for v in a.freqs.split(",")]
    a.f0 = freqs[0]
    if a.no_vsg:
        hwtest.VSG = NoVSG
        if a.gain is None: a.gain = 50
    t = PN(a); r = t.r
    try:
        r.emu.cmd("set spec=0 rate=250000 freq=%d gain=%d" % (a.f0, a.gain or 40))
        r.emu.start(); r.emu.cmd("sync 8")
        r.rec(test="start", args=vars(a))
        if a.gain is None: r.guarded("gain", r.pick_gain, a.f0)
        for f0 in freqs:
            a.f0 = f0
            if not a.no_vsg: r.guarded("ppm", r.calibrate)
            for k, fn in (("P", t.test_p), ("R", t.test_r), ("F", t.test_f)):
                if k in a.tests: r.guarded(k, fn, f0)
        r.rec(test="end", **r.counters())
        r.say("done: %s" % r.counters())
    finally:
        try: t.fm_on(False)
        except Exception: pass
        r.close()
    plots(a.out)


if __name__ == "__main__":
    main()
