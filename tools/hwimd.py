"""Conducted two-tone IMD3 / IIP3 and blocking (reciprocal mixing) for the ESP32-S3 and external DUTs.
    python tools/hwimd.py --dut hackrf --atten 11.2 --gains 40:20,32:20,24:20,16:20 [--phases IK] [--vsg-max -50]
    python tools/hwimd.py --dut esp32s3 --atten 11.2 --gains 70,50,30
I  two tones 20 kHz apart (VSG60 multitone) at f0+35/55 kHz, 250 kS/s. The input level of each tone comes from a CW
   gain calibration (tone dBFS - input dBm) at the same gain, so the VSG's multitone power split does not matter.
   IIP3 = P_tone + (P_tone - P_IM3) / 2, from the points where the IM3 products are >= 6 dB above the noise.
K  CW blocker at +-0.2 .. +-20 MHz: rise of the in-band noise density, converted to reciprocal-mixing noise in dBc/Hz
   relative to the blocker (= the LO's far-out phase noise when the floor rise is reciprocal mixing).
Plots open on screen. The VSG never goes above --vsg-max."""
import argparse, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hwcampaign import Camp
from hwtest import iq_analyze
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

RATE = 250000
CFO, SP = 45e3, 20e3                  # two-tone centre offset and spacing: tones +35 / +55 kHz, IM3 +15 / +75 kHz
BW = 150.0                            # +- Hz window around each tone / product
SEARCH = 6000.0                       # tone search +- Hz: the DUT's own frequency error (HackRF +2.1 kHz, S3 -4.9 kHz)


def psd(x):
    n = len(x); w = np.hanning(n)
    P = np.abs(np.fft.fftshift(np.fft.fft(x * w))) ** 2 / (n * np.sum(w ** 2))
    return P, (np.arange(n) - n // 2) * RATE / n


def band_pow(P, fx, f):
    sel = np.abs(fx - f) <= BW
    return 10 * np.log10(P[sel].sum() + 1e-30)


class Imd(Camp):
    ppm = {}                                          # per-frequency ppm correction found by lock()

    def tune(self, f0, rate, gain, settle=0.4):
        g = int(gain) if self.a.dut == "esp32s3" else gain
        self.e.set(spec=0, rate=rate, freq=int(f0), gain=g, ppm="%.3f" % self.ppm.get(f0, 0.0), settle=settle)

    def lock(self, f0, g):
        """S3 only: find the frequency error with a CW at +0.2 * rate (searched over the whole band) and correct it
        with the ppm setting, so the test tones land where the analysis expects them (error -1.8 ppm, plus warm-up)."""
        if self.a.dut != "esp32s3": return
        off = 0.2 * RATE

        def meas(ppm):
            self.ppm[f0] = ppm; self.tune(f0, RATE, g)
            self.vset(f=f0 + off, p=-70, on=True); time.sleep(0.2)
            P, fx = psd(self.cap(RATE, 0.5)); k = (np.abs(fx) > 500) & (np.abs(fx) < 0.45 * RATE)
            return float(fx[k][np.argmax(P[k])] - off)
        p0 = self.ppm.get(f0, 0.0); err = meas(p0)
        if abs(err) > 150:
            d = err / f0 * 1e6
            e1 = meas(p0 + d)
            if abs(e1) > abs(err): e1 = meas(p0 - d)       # the emulator's ppm sign is the other way round
            err = e1
        self.vset(on=False)
        self.say("lock %.0f MHz: ppm %.3f, residual %.0f Hz" % (f0 / 1e6, self.ppm.get(f0, 0.0), err))

    def cap(self, rate, secs, path=None):
        for attempt in range(3):                      # the S3 link can drop a capture right after a retune: retry
            x = Camp.cap(self, rate, secs, path)
            if x is not None and len(x): return x
            self.say("capture empty (%s), retry %d" % (self.e.stats(), attempt + 1)); time.sleep(0.5)
        raise RuntimeError("no samples from %s" % self.a.dut)

    def two_tone(self, on, cf=None, p=None):
        if on:
            self.v.w(":SOURce:MTONe:NTONes 2"); self.v.w(":SOURce:MTONe:FSPacing %d" % int(SP))
            self.v.w(":SOURce:MTONe:PHASe FIXed"); self.vset(f=cf, p=p)
            self.v.w(":SOURce:MTONe:STATe ON"); self.v.w(":OUTPut:MODulation:STATe ON"); self.v.set(on=True)
        else:
            self.v.w(":SOURce:MTONe:STATe OFF"); self.v.w(":OUTPut:MODulation:STATe OFF"); self.v.set(on=False)
            self.v.w(":SOURce:POWer -50.00")

    def vset(self, **k):
        # hwtest.VSG.set() clamps the level to -50 dBm; here the ceiling is --vsg-max instead
        if "p" in k:
            p = min(float(k.pop("p")), self.a.vsg_max, -10.0)
            self.v.w(":SOURce:POWer %.2f" % p)
        self.v.set(**k)

    def cw_cal(self, f0, g, p_vsg=-60.0):
        """tone dBFS (in the +-BW window) minus input dBm at this gain."""
        self.vset(f=f0 + CFO - SP / 2, p=p_vsg, on=True); time.sleep(0.15)
        x = self.cap(RATE, 0.5); P, fx = psd(x)
        k = np.abs(fx - (CFO - SP / 2)) < SEARCH; fpk = fx[k][np.argmax(P[k])]
        return band_pow(P, fx, fpk) - (p_vsg - self.a.atten), float(np.max(np.abs(x)))

    def phase_i(self, f0):
        self.lock(f0, 60)
        lo, hi, st = self.a.ilevels
        levels = np.arange(lo, min(hi, self.a.vsg_max) + 0.01, abs(st))      # weak -> strong, stops at ADC clipping
        fig, axs = plt.subplots(1, len(self.gains), figsize=(4.6 * len(self.gains), 4.6), squeeze=False)
        summ = []
        for gi, g in enumerate(self.gains):
            self.tune(f0, RATE, g)
            cal, pk = self.cw_cal(f0, g, self.a.cal_p)
            self.vset(on=False); time.sleep(0.1); x = self.cap(RATE, 0.5); P0, fx = psd(x)
            floor = 10 * np.log10(np.median(P0) * np.sum(np.abs(fx - 15e3) <= BW))   # noise in one window
            self.two_tone(True, f0 + CFO, levels[0])
            rows = []
            for p in levels:
                self.vset(p=float(p)); time.sleep(0.12)
                x = self.cap(RATE, 0.5); P, fx = psd(x)
                peak = getattr(self.e, "peak_raw", None) or float(np.max(np.abs(x)))   # HackRF: 8-bit ADC before decimation
                if peak > self.a.clip:
                    self.say("I %s: ADC peak %.2f FS at VSG %.0f dBm -> sweep stopped" % (g, peak, p)); break
                k = np.abs(fx - (CFO - SP / 2)) < SEARCH; d = fx[k][np.argmax(P[k])] - (CFO - SP / 2)
                f1, f2 = CFO - SP / 2 + d, CFO + SP / 2 + d
                t = 0.5 * (band_pow(P, fx, f1) + band_pow(P, fx, f2))
                im = 10 * np.log10(0.5 * (10 ** (band_pow(P, fx, 2 * f1 - f2) / 10) + 10 ** (band_pow(P, fx, 2 * f2 - f1) / 10)))
                rows.append((float(p), t - cal, im - cal, im - floor, peak))
                self.rec(test="I", f0=f0, gain=str(g), p_vsg=float(p), cal=cal, tone_dbfs=t, im3_dbfs=im, floor_dbfs=floor,
                         p_tone=t - cal, p_im3=im - cal, peak=peak)
            self.two_tone(False)
            r = np.array(rows)
            if not len(r):
                self.say("I %s: no point below the clip level" % (g,)); continue
            # usable: IM3 >= 6 dB above the noise AND >= 10 dB above the generator's own IM3 (tone - src_dbc)
            ok = (r[:, 3] >= 6) & (r[:, 2] >= r[:, 1] - self.a.src_dbc + 10)
            iip3 = r[ok, 1] + (r[ok, 1] - r[ok, 2]) / 2 if ok.any() else np.array([])
            iip3_med = float(np.median(iip3)) if len(iip3) else float("nan")
            # IM3 slope where measurable (should be ~3 if it is real third-order distortion)
            slope = float(np.polyfit(r[ok, 1], r[ok, 2], 1)[0]) if ok.sum() >= 3 else float("nan")
            summ.append((g, cal, floor - cal, iip3_med, slope, int(ok.sum()), float(r[:, 4].max())))
            self.rec(test="I_sum", f0=f0, gain=str(g), cal=cal, floor_in=floor - cal, iip3=iip3_med, slope=slope,
                     n_ok=int(ok.sum()), max_peak=float(r[:, 4].max()))
            self.say("I %s: IIP3 %s dBm (slope %.2f, %d points above floor), noise in %d Hz %.1f dBm, max peak %.2f FS"
                     % (g, "%.1f" % iip3_med if len(iip3) else "not reached", slope, ok.sum(), 2 * BW, floor - cal, r[:, 4].max()))
            ax = axs[0][gi]
            ax.plot(r[:, 1], r[:, 1], "o-", ms=3, label="tone")
            ax.plot(r[:, 1], r[:, 2], "s-", ms=3, label="IM3 (input-referred)")
            ax.axhline(floor - cal, color="gray", ls=":", label="noise, %d Hz" % (2 * BW))
            ax.plot(r[:, 1], r[:, 1] - self.a.src_dbc, color="C3", ls=":", lw=1, label="VSG60 own IM3 (-%.0f dBc)" % self.a.src_dbc)
            if len(iip3):
                xx = np.linspace(r[:, 1].min(), iip3_med, 20)
                ax.plot(xx, xx, "C0--", lw=0.7); ax.plot(xx, 3 * xx - 2 * iip3_med, "C1--", lw=0.7)
                ax.plot([iip3_med], [iip3_med], "k*", ms=9)
            ax.set_title("%s gain %s: IIP3 %s" % (self.a.dut, g, "%.1f dBm" % iip3_med if len(iip3) else "not reached"))
            ax.set_xlabel("tone level at input [dBm]"); ax.set_ylabel("level at input [dBm]"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
        self.show(fig, "I_imd3.png")
        return summ

    def phase_k(self, f0):
        """Desensitisation: in-band noise density with a CW blocker, vs blocker offset and level, at the sensitivity gain.
        recip = excess noise density relative to the blocker [dBc/Hz] (reciprocal mixing, or the VSG's own far-out noise)."""
        g = self.parse_gain(self.a.kgain) if self.a.kgain else self.gains[0]
        plev = sorted(min(float(v), self.a.vsg_max) for v in self.a.klevels.split(","))
        self.lock(f0, 60)
        self.tune(f0, RATE, g)
        cal, _ = self.cw_cal(f0, g, self.a.cal_p)
        self.vset(on=False); time.sleep(0.1)

        def nd(x):                                        # median noise density in the band, away from DC and the edges
            P, fx = psd(x); sel = (np.abs(fx) > 0.04 * RATE) & (np.abs(fx) < (0.4 if RATE > 1e5 else 0.25) * RATE)
            return 10 * np.log10(np.median(P[sel]) / (RATE / len(x)))   # dBFS/Hz
        n0 = nd(self.cap(RATE, 1.0)); pk0 = getattr(self.e, "peak_raw", None)
        offs = (0.2e6, 0.3e6, 0.5e6, 1e6, 2e6, 3e6, 5e6, 10e6, 20e6)
        rows = []
        for off in offs:
            for sgn in (1, -1):
                for p in plev:
                    self.vset(f=f0 + sgn * off, p=p, on=True); time.sleep(0.15)
                    n1 = nd(self.cap(RATE, 1.0)); pk = getattr(self.e, "peak_raw", None)
                    pb = p - self.a.atten; rise = n1 - n0
                    rm = 10 * np.log10(max(10 ** (n1 / 10) - 10 ** (n0 / 10), 1e-30)) - cal - pb if rise > 1.0 else float("nan")
                    rows.append((sgn * off, pb, rise, rm, pk if pk is not None else float("nan")))
                    self.rec(test="K", f0=f0, gain=str(g), off=sgn * off, p_blocker=pb, n0=n0, n1=n1, rise=rise,
                             recip_dbc_hz=rm, adc_peak=pk)
                self.vset(on=False); time.sleep(0.05)
        self.vset(on=False)
        r = np.array(rows)
        for pb in sorted(set(r[:, 1])):
            k = r[:, 1] == pb
            self.say("K %s, blocker %.0f dBm: rise %s" % (g, pb, " ".join("%+.1fM:%.1f" % (o / 1e6, x) for o, x in r[k][:, [0, 2]])))
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.4))
        for i, pb in enumerate(sorted(set(r[:, 1]))):
            k = r[:, 1] == pb
            for s, m in ((1, "-"), (-1, "--")):
                kk = k & (np.sign(r[:, 0]) == s); o = np.abs(r[kk, 0]) / 1e6
                lab = "%.0f dBm" % pb if s > 0 else None
                ax[0].semilogx(o, r[kk, 2], "C%d%s" % (i, m), marker="o", ms=3, label=lab)
                ax[1].semilogx(o, r[kk, 3], "C%d%s" % (i, m), marker="o", ms=3, label=lab)
        ax[0].set_title("%s: in-band noise rise with a CW blocker, gain %s (dashed: blocker below)" % (self.a.dut, g), fontsize=10)
        ax[0].set_ylabel("noise floor rise [dB]")
        ax[1].set_title("excess noise relative to the blocker (rise > 1 dB only)", fontsize=10); ax[1].set_ylabel("dBc/Hz")
        for a_ in ax: a_.set_xlabel("blocker offset [MHz]"); a_.grid(alpha=0.3, which="both"); a_.legend(fontsize=8, title="blocker")
        self.show(fig, "K_blocking.png")

    def phase_n(self, freqs):
        """Noise figure and clip-limited dynamic range vs gain: CW gain calibration, then the VSG off.
        NF = input-referred noise density + 174 dBm/Hz (median of the periodogram, +1.59 dB for the exponential bias);
        the source is the 11.2 dB pad (290 K) in front of the VSG. Clip level = CW input that would reach ADC full scale."""
        rows = []
        for f0 in freqs:
            self.lock(f0, 60)
            for g in self.gains:
                self.tune(f0, RATE, g)
                cal, _ = self.cw_cal(f0, g, self.a.cal_p)
                pk = getattr(self.e, "peak_raw", None)
                self.vset(on=False); time.sleep(0.15)
                x = self.cap(RATE, 1.0); P, fx = psd(x)
                sel = (np.abs(fx) > 0.04 * RATE) & (np.abs(fx) < (0.4 if RATE > 1e5 else 0.25) * RATE)   # S3 DDC passband
                nd = 10 * np.log10(np.median(P[sel]) / (RATE / len(x))) + 1.59          # dBFS/Hz
                nin = nd - cal; nf = nin + 174.0
                clip = (self.a.cal_p - self.a.atten) - 20 * np.log10(pk) if pk else float("nan")
                sfdr_like = clip - (nin + 10 * np.log10(RATE))                          # clip level over noise in 250 kHz
                rows.append((f0, g, nf, clip, sfdr_like))
                self.rec(test="N", f0=f0, gain=str(g), cal=cal, noise_dbm_hz=nin, nf=nf, clip_dbm=clip, dr_250k=sfdr_like)
                self.say("N %.0f MHz gain %s: NF %.1f dB, ADC full scale at %.1f dBm, %.1f dB over the noise in 250 kHz"
                         % (f0 / 1e6, g, nf, clip, sfdr_like))
        self.vset(on=False)
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.4))
        for i, f0 in enumerate(freqs):
            rr = [r for r in rows if r[0] == f0]
            lab = [str(r[1]) for r in rr]
            ax[0].plot(range(len(rr)), [r[2] for r in rr], "o-", label="%.0f MHz" % (f0 / 1e6))
            ax[1].plot(range(len(rr)), [r[3] for r in rr], "o-", label="%.0f MHz" % (f0 / 1e6))
        for a_, t in zip(ax, ("noise figure [dB]", "CW input at ADC full scale [dBm]")):
            a_.set_xticks(range(len(lab))); a_.set_xticklabels(lab, rotation=45, fontsize=8); a_.set_ylabel(t)
            a_.grid(alpha=0.3); a_.legend(fontsize=8); a_.set_xlabel("gain (%s)" % ("LNA, VGA" if self.a.dut == "hackrf" else "index"))
        ax[0].set_title("%s noise figure vs gain (conducted, 11.2 dB pad as source)" % self.a.dut, fontsize=10)
        ax[1].set_title("%s clip level vs gain" % self.a.dut, fontsize=10)
        self.show(fig, "N_noise_figure.png")

    def phase_q(self, f_lo=2230e6, f_hi=2780e6, step=50e6, offs=(20e3, 40e3, 100e3)):
        """IQ image rejection and DC offset vs LO frequency: CW at +off, image power at -off (same +-BW window);
        DC = power within +-200 Hz of 0 Hz with the VSG off, relative to the in-band noise in the same window."""
        g = self.gains[0]; rows = []
        for f0 in np.arange(f_lo, f_hi + 1, step):
            f0 = float(f0); self.lock(f0, g)
            self.tune(f0, RATE, g)
            self.vset(on=False); time.sleep(0.1)
            x = self.cap(RATE, 0.5); P, fx = psd(x)
            dc = 10 * np.log10(P[np.abs(fx) <= 200].sum() + 1e-30)
            nwin = 10 * np.log10(np.median(P[(np.abs(fx) > 5e3) & (np.abs(fx) < 1e5)]) * np.sum(np.abs(fx) <= 200))
            for off in offs:
                self.vset(f=f0 + off, p=self.a.cal_p, on=True); time.sleep(0.12)
                x = self.cap(RATE, 0.5); P, fx = psd(x)
                k = np.abs(fx - off) < SEARCH; fp = fx[k][np.argmax(P[k])]
                t = band_pow(P, fx, fp); im = band_pow(P, fx, -fp)
                rows.append((f0, off, im - t, dc - nwin, t))
                self.rec(test="Q", f0=float(f0), gain=str(g), off=off, tone_dbfs=t, image_dbc=im - t, dc_over_noise=dc - nwin)
            self.say("Q %.0f MHz: image %s dBc, DC %.1f dB over the noise (+-200 Hz)" % (f0 / 1e6,
                     " / ".join("%.1f" % r[2] for r in rows[-len(offs):]), dc - nwin))
        self.vset(on=False)
        r = np.array(rows)
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
        for i, off in enumerate(offs):
            k = r[:, 1] == off
            ax[0].plot(r[k, 0] / 1e6, r[k, 2], "o-", label="tone at +%.0f kHz" % (off / 1e3))
        k = r[:, 1] == offs[0]
        ax[1].plot(r[k, 0] / 1e6, r[k, 3], "o-")
        ax[0].set_title("%s IQ image (gain %s), tone %.0f dBm" % (self.a.dut, g, self.a.cal_p - self.a.atten), fontsize=10)
        ax[0].set_ylabel("image [dBc]"); ax[1].set_ylabel("DC spike over noise, +-200 Hz [dB]")
        ax[1].set_title("%s DC offset (VSG off)" % self.a.dut, fontsize=10)
        for a_ in ax: a_.set_xlabel("LO frequency [MHz]"); a_.grid(alpha=0.3)
        ax[0].legend(fontsize=8)
        self.show(fig, "Q_image_dc.png")

    def parse_gain(self, s):
        return tuple(int(v) for v in s.split(":")) if self.a.dut == "hackrf" else int(s)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--emu", default=os.path.join(here, "..", "build", "Release", "esp_sdr_emu.exe"))
    ap.add_argument("--out", default=time.strftime("imd_%Y%m%d_%H%M"))
    ap.add_argument("--atten", type=float, required=True)
    ap.add_argument("--dut", default="esp32s3", choices=["esp32s3", "hackrf", "rtlsdr", "bb60c"])
    ap.add_argument("--gains", required=True, help="esp32s3: 70,50,30  hackrf: lna:vga[:amp],...")
    ap.add_argument("--phases", default="IK")
    ap.add_argument("--f0", type=float, default=2350e6)
    ap.add_argument("--vsg-max", type=float, default=-50.0)
    ap.add_argument("--ilevels", default="-100,-50,2", help="two-tone VSG sweep lo,hi,step (use --ilevels=...)")
    ap.add_argument("--cal-p", type=float, default=-60.0, help="VSG level of the CW gain calibration")
    ap.add_argument("--clip", type=float, default=0.7, help="stop a sweep above this ADC peak [FS]")
    ap.add_argument("--src-dbc", type=float, default=42.0, help="generator's own two-tone IM3 [dBc], measured")
    ap.add_argument("--kgain", default=None, help="gain for K (default: first of --gains)")
    ap.add_argument("--klevels", default="-50,-40,-30,-20,-14", help="blocker VSG levels")
    ap.add_argument("--freqs", default="2350e6", help="phase N frequencies, comma-separated Hz")
    ap.add_argument("--rate", type=int, default=250000, help="IQ rate; 62500 = the S3's 16-bit link (tones then at +5 / +15 kHz)")
    ap.add_argument("--warmup", type=float, default=60.0, help="S3: seconds of streaming before the first measurement")
    ap.add_argument("--hackrf-fs", type=float, default=None, help="HackRF native rate (default 2 MS/s; 10e6 keeps blockers <= 5 MHz from aliasing in)")
    a = ap.parse_args()
    a.ilevels = [float(v) for v in a.ilevels.split(",")]
    global RATE, CFO, SP
    RATE = a.rate
    # 62.5k (S3 16-bit link): tones at +5 / +15 kHz, IM3 at -5 / +25 kHz (VSG60 multitone needs >= 10 kHz spacing);
    # lock() corrects the S3's frequency error first, so they land there
    if RATE < 100000: CFO, SP = 10e3, 10e3
    if a.hackrf_fs:
        import pn_ext
        pn_ext.DUTS["hackrf"]["fs"] = int(a.hackrf_fs)
    c = Imd(a)
    c.gains = [c.parse_gain(s) for s in a.gains.split(",")]
    c.say("hwimd %s, atten %.1f dB, VSG max %.0f dBm, gains %s" % (a.dut, a.atten, a.vsg_max, c.gains))
    try:
        if a.dut == "esp32s3":
            c.e.cmd("set spec=0 rate=%d freq=%d gain=%d" % (RATE, a.f0, c.gains[0])); c.e.start(); c.e.cmd("sync 8")
            c.say("S3 warm-up: streaming %d s before measuring" % a.warmup); time.sleep(a.warmup)
        if "I" in a.phases: c.phase_i(a.f0)
        if "K" in a.phases: c.phase_k(a.f0)
        if "N" in a.phases: c.phase_n([float(v) for v in a.freqs.split(",")])
        if "Q" in a.phases: c.phase_q(offs=(20e3, 40e3, 100e3) if RATE > 100000 else (5e3, 10e3, 20e3))
    finally:
        try: c.two_tone(False)
        except Exception: pass
        c.v.set(on=False)
        open(os.path.join(a.out, "SUMMARY.txt"), "w", encoding="utf-8").write("\n".join(c.notes) + "\n")
        try: c.e.quit()
        except Exception: pass


if __name__ == "__main__":
    main()
