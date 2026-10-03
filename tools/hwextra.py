"""Receiver characterisation beyond hwlong.py (radiated / near-field setup, so levels are relative):
    python tools/hwextra.py [--tests TIKMQ] [--adev-secs 3600] [--out DIR] [--port COM4]
T  ADEV: gapless IQ capture of a CW, phase-based Allan deviation tau = 0.01 .. ~600 s (S3 crystal vs VSG reference)
I  two-tone IMD3 (VSG60 multitone): IM3 in dBc vs gain index and vs VSG level, both IQ links
K  blocking / reciprocal mixing: in-band noise-floor rise and spurs with a CW blocker at +-0.2 .. +-20 MHz
M  latency: VSG output on -> first affected sample / spectrum arriving at the host (IQ 250k, 62.5k, SPEC)
Q  IQ imbalance vs LO frequency: SPEC mirror and IQ image across 2230-2780 MHz
E  EVM (needs a QAM signal set up by hand in Spike, see --evm-symrate); not run unless given"""
import argparse, os, sys, time, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hwtest import Emu, iq_analyze, spec_load, spec_mean, spec_peak, MHZ
from hwlong import Long


class Extra(Long):
    # ---------------- T: Allan deviation ----------------
    def test_t(self, secs):
        rate, off, f0 = 62500, 10e3, self.a.f0
        self.vsg.set(f=f0 + off, p=-50, on=True)
        self.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.gain, settle=1.0)
        path = os.path.join(self.a.out, "adev_iq.cf32")
        self.say("T: %.0f s gapless capture at %d S/s -> %s" % (secs, rate, path))
        s0 = self.emu.stats()
        ok, got, dt, _ = self.emu.cap(int(rate * secs), path, timeout=secs + 120)
        s1 = self.emu.stats()
        # phase of the tone, decimated to 100 Hz
        x = np.memmap(path, np.complex64, "r")
        blk = rate // 100; nb = len(x) // blk
        w = np.exp(-2j * np.pi * off * np.arange(blk) / rate)
        z = np.empty(nb, np.complex128)
        for c0 in range(0, nb, 20000):            # chunked: 20000 blocks = 200 s
            c1 = min(nb, c0 + 20000)
            X = np.asarray(x[c0 * blk:c1 * blk], np.complex128).reshape(c1 - c0, blk)
            k = np.arange(c0, c1)
            z[c0:c1] = (X * w).sum(axis=1) * np.exp(-2j * np.pi * off * blk * k / rate)
        ph = np.unwrap(np.angle(z))
        xt = ph / (2 * np.pi * f0)              # time error [s] of the received carrier vs nominal
        taus, adev = [], []
        t0 = 0.01; m = 1
        while m * 3 < nb:
            d = xt[2 * m:] - 2 * xt[m:-m] + xt[:-2 * m]   # overlapping ADEV from phase data
            taus.append(m * t0); adev.append(float(np.sqrt(np.mean(d ** 2) / (2 * (m * t0) ** 2))))
            m = int(np.ceil(m * 1.6))
        tt = np.arange(nb) * t0
        q = np.polyfit(tt, ph / (2 * np.pi), 2)   # cycles = c + f t + D/2 t^2
        self.rec(test="T", secs=secs, samples=got, ok=ok, tau=taus, adev=adev, mean_freq_err_hz=float(q[1] + q[0] * tt[-1]),
                 drift_hz_per_hour=float(2 * q[0] * 3600), f0=f0,
                 crc=s1["crc"] - s0["crc"], gaps=s1["gaps"] - s0["gaps"], lost=s1["lost"] - s0["lost"])
        dec = np.arange(0, nb, 100)
        np.save(os.path.join(self.a.out, "adev_phase_1s.npy"), ph[dec])
        del x
        try: os.remove(path)
        except OSError: pass
        self.say("T: ADEV(1 s) %.2e, ADEV(100 s) %.2e" % (np.interp(1, taus, adev), np.interp(100, taus, adev)))

    # ---------------- I: two-tone IMD3 ----------------
    def two_tone(self, on, cf=None, spacing=None):
        if on:
            self.vsg.w(":SOURce:MTONe:NTONes 2"); self.vsg.w(":SOURce:MTONe:FSPacing %d" % int(spacing))
            self.vsg.w(":SOURce:MTONe:PHASe FIXed")
            self.vsg.set(f=cf, p=-50)
            # VSG60 / Spike: multitone only reaches the output with the modulation path enabled
            self.vsg.w(":SOURce:MTONe:STATe ON"); self.vsg.w(":OUTPut:MODulation:STATe ON"); self.vsg.set(on=True)
        else:
            self.vsg.w(":SOURce:MTONe:STATe OFF"); self.vsg.w(":OUTPut:MODulation:STATe OFF"); self.vsg.set(p=-50)

    IMD_BW_HZ = 150   # +- window around each tone / product, after locking onto the measured tone position

    def tone_pow(self, P, fs, at):
        n = len(P); bw = max(4, int(self.IMD_BW_HZ / (fs / n))); k = int(round(at / (fs / n))) + n // 2
        return 10 * np.log10(P[max(0, k - bw):k + bw + 1].sum() + 1e-30)

    def imd_capture(self, rate, secs, f1, f2):
        ok, got, dt, _ = self.emu.cap(int(rate * secs), self.tmp, timeout=secs * 4 + 3)
        x = np.fromfile(self.tmp, np.complex64).astype(np.complex128); n = len(x)
        w = np.hanning(n)
        P = np.abs(np.fft.fftshift(np.fft.fft(x * w))) ** 2 / (n * np.sum(w ** 2))
        # lock onto the actual tone position (residual crystal drift moves everything by the same delta)
        fx = (np.arange(n) - n // 2) * rate / n
        sel = np.abs(fx - f1) < 1500
        delta = float(fx[sel][np.argmax(P[sel])] - f1) if np.any(sel) else 0.0
        f1, f2 = f1 + delta, f2 + delta
        t1, t2 = self.tone_pow(P, rate, f1), self.tone_pow(P, rate, f2)
        lo, hi = self.tone_pow(P, rate, 2 * f1 - f2), self.tone_pow(P, rate, 2 * f2 - f1)
        nb = 2 * max(4, int(self.IMD_BW_HZ / (rate / n))) + 1
        floor = 10 * np.log10(np.median(P) * nb + 1e-30)         # noise in the same window as tone_pow
        return dict(t1=t1, t2=t2, im3_lo=lo, im3_hi=hi, floor9=floor, peak=float(np.max(np.abs(x))), delta_hz=delta)

    def test_i(self):
        f0 = self.a.f0
        # (rate, VSG centre offset, spacing): tones and both IM3 products inside the band, away from 0 Hz
        # VSG60 multitone needs >= 10 kHz spacing (5 kHz gives no output)
        cfgs = ((250000, 45e3, 20e3), (62500, 10e3, 10e3))
        for rate, cfo, sp in cfgs:
            f1, f2 = cfo - sp / 2, cfo + sp / 2
            self.two_tone(True, f0 + cfo, sp)
            for g in range(10, 83, 4):
                self.emu.set(spec=0, rate=rate, freq=int(f0), gain=g, settle=0.15)
                r = self.imd_capture(rate, 1.0 if rate < 1e5 else 0.5, f1, f2)
                self.rec(test="I", sweep="gain", rate=rate, gain=g, p=-50, spacing=sp, **r)
            for p in np.arange(-50, -81, -2):
                self.vsg.set(p=p); time.sleep(0.1)
                self.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.gain, settle=0.1)
                r = self.imd_capture(rate, 1.0 if rate < 1e5 else 0.5, f1, f2)
                self.rec(test="I", sweep="power", rate=rate, gain=self.gain, p=float(p), spacing=sp, **r)
            self.two_tone(False)
            self.say("I %d done" % rate)
        self.emu.set(spec=0, rate=250000, gain=self.gain, settle=0.1)

    # ---------------- K: blocking / reciprocal mixing ----------------
    def test_k(self):
        f0 = self.a.f0
        for rate in (250000, 62500):
            self.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.gain, settle=0.3)
            self.vsg.set(on=False); time.sleep(0.2)
            ok, r0, _ = self.iq(rate, secs=1.0)
            ref = r0["floor_bin"]
            self.rec(test="K", rate=rate, off=0, vsg=False, floor_bin=ref, n=r0["n"])
            # -4 MHz is the LO, -8 MHz the image of the fs/4 IF (mirror of f about the LO): shows the true image rejection
            for off in (0.2e6, 0.5e6, 1e6, 2e6, 3e6, 4e6, 5e6, 8e6, 10e6, 20e6):
                for sgn in (1, -1):
                    self.vsg.set(f=f0 + sgn * off, p=-50, on=True); time.sleep(0.1)
                    ok, r, _ = self.iq(rate, secs=1.0)
                    self.rec(test="K", rate=rate, off=sgn * off, vsg=True, floor_bin=r["floor_bin"], rise=r["floor_bin"] - ref,
                             strongest=r["level"], strongest_freq=r["freq"], n=r["n"])
            self.vsg.set(on=False)
            self.say("K %d done" % rate)
        self.emu.set(spec=0, rate=250000, gain=self.gain, settle=0.1)

    # ---------------- M: latency VSG on -> data at the host ----------------
    def test_m(self, reps=15):
        f0 = self.a.f0
        for mode in ((0, 250000, 0), (0, 62500, 0), (80e6, 0, 256), (80e6, 0, 1024)):
            fs, rate, bins = mode
            if fs:
                off = 10e6; self.emu.set(spec=int(fs), bins=bins, freq=int(f0), gain=self.gain, dcfix=1, settle=0.5)
            else:
                off = 0.16 * rate; self.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.gain, settle=0.3)
            self.vsg.set(f=f0 + off, p=-50, on=False)
            for i in range(reps):
                self.vsg.set(on=False); time.sleep(0.3)
                n = int(rate * 1.2) if not fs else 60
                t_req = time.perf_counter()
                self.emu.p.stdin.write("cap %d %s %g 1\n" % (n, self.tmp, 10)); self.emu.p.stdin.flush()
                time.sleep(0.4)
                t_on = time.perf_counter(); self.vsg.w(":OUTPut:STATe ON"); t_on2 = time.perf_counter()
                r = self.emu.p.stdout.readline().strip()
                ts = np.fromfile(self.tmp + ".ts", dtype=[("t", "<f8"), ("n", "<i8")])
                clk = ts["t"][0] - t_req                     # emu steady clock minus perf_counter (~0)
                if fs:
                    S = spec_load(self.tmp, bins); j0 = int(round(bins / 2 + off / (fs / bins)))
                    lv = S[:, max(0, j0 - 2):j0 + 3].max(axis=1); base = np.median(lv[:5])
                    idx = int(np.argmax(lv > base + 15)) if np.any(lv > base + 15) else -1
                else:
                    x = np.fromfile(self.tmp, np.complex64).astype(np.complex128)
                    seg = max(16, rate // 2000)              # 0.5 ms slices
                    yy = x[: len(x) // seg * seg].reshape(-1, seg) * np.exp(-2j * np.pi * off * np.arange(len(x) // seg * seg).reshape(-1, seg) / rate)
                    pw = np.abs(yy.mean(axis=1)) ** 2; base = np.median(pw[:50]) + 1e-30
                    hit = np.where(pw > base * 100)[0]
                    idx = int(hit[0] * seg) if len(hit) else -1
                if idx < 0:
                    self.rec(test="M", mode=list(mode), i=i, found=False); continue
                k = int(np.searchsorted(ts["n"], idx + 1))   # first callback that delivered that sample/spectrum
                t_arr = ts["t"][min(k, len(ts) - 1)] - clk
                self.rec(test="M", mode=list(mode), i=i, found=True, latency_ms=(t_arr - t_on) * 1e3,
                         scpi_ms=(t_on2 - t_on) * 1e3, clk_offset_ms=clk * 1e3, callbacks=int(len(ts)))
            self.vsg.set(on=False)
            self.say("M %s done" % (mode,))
        self.emu.set(spec=0, rate=250000, gain=self.gain, settle=0.1)

    # ---------------- Q: IQ imbalance vs LO frequency ----------------
    def test_q(self):
        for c in np.arange(2230e6, 2780e6 + 1, 50e6):
            self.emu.set(spec=80000000, bins=1024, freq=int(c), gain=self.gain, dcfix=1, settle=0.3)
            for off in (5e6, 10e6, 20e6, 30e6):
                self.vsg.set(f=c + off, p=-50, on=True); time.sleep(0.05)
                self.emu.cap(3, self.tmp, timeout=3)
                ok, S, _ = self.spec(80e6, 1024, 20)
                if S is None or not len(S): continue
                q = spec_peak(spec_mean(S), 80e6, off, search_bins=3)
                self.rec(test="Q", f=float(c), off=off, level=q["level"], mirror_db=q["mirror_db"], median=q["median"])
            self.vsg.set(f=c + 40e3, on=True)
            self.emu.set(spec=0, rate=250000, freq=int(c), gain=self.gain, settle=0.15)
            ok, r, _ = self.iq(250000, f_exp=40e3, search=3000)
            if r: self.rec(test="Qiq", f=float(c), image_dbc=r["image_dbc"], level=r["level"])
        self.vsg.set(on=False)
        self.emu.set(spec=0, rate=250000, freq=int(self.a.f0), gain=self.gain, settle=0.1)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--emu", default=os.path.join(here, "..", "build", "Release", "esp_sdr_emu.exe"))
    ap.add_argument("--out", default=time.strftime("hwextra_%Y%m%d_%H%M"))
    ap.add_argument("--f0", type=float, default=2350e6)
    ap.add_argument("--gain", type=int, default=None)
    ap.add_argument("--tests", default="QKIMT")
    ap.add_argument("--adev-secs", type=float, default=3600)
    ap.add_argument("--soak", type=float, default=0)
    ap.add_argument("--hours", type=float, default=0)
    a = ap.parse_args()
    r = Extra(a)
    r.t_start = time.time()
    try:
        r.emu.cmd("set spec=0 rate=250000 freq=%d gain=%d" % (a.f0, a.gain or 40))
        r.emu.start(); r.emu.cmd("sync 8")
        r.rec(test="start", args=vars(a))
        if a.gain is None: r.guarded("gain", r.pick_gain, a.f0)
        for k, fn, args in (("Q", r.test_q, ()), ("K", r.test_k, ()), ("I", r.test_i, ()), ("M", r.test_m, ()),
                            ("T", r.test_t, (a.adev_secs,))):
            if k in a.tests:
                r.guarded("ppm", r.calibrate); r.guarded(k, fn, *args)
        r.rec(test="end", **r.counters())
        r.say("done: %s" % r.counters())
    finally:
        try: r.two_tone(False)
        except Exception: pass
        r.close()


if __name__ == "__main__":
    main()
