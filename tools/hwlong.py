"""Long hardware campaign on top of hwtest.py: the standard tests, extra characterisation, then a long soak
until the deadline.     python tools/hwlong.py --hours 6 [--out DIR] [--port COM4]
G gain curve | L level linearity / MDS | N noise floor vs gain | R SPEC frame rates of every profile |
S retune settling | F phase noise | W long soak (IQ / SPEC blocks with checks, hourly mini sweeps and stress)
Every test is wrapped: an exception is logged and the campaign continues (the emulator is restarted if needed)."""
import argparse, os, sys, time, traceback, random
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hwtest import Run, Emu, iq_analyze, spec_load, spec_mean, spec_peak, out_shift, MHZ


class Long(Run):
    def restart_emu(self):
        try: self.emu.quit()
        except Exception: pass
        time.sleep(2)
        self.emu = Emu(self.a.emu, self.a.port)
        self.emu.cmd("set spec=0 rate=250000 freq=%d gain=%d ppm=%.3f" % (self.a.f0, self.gain, self.ppm))
        self.emu.start(); self.emu.cmd("sync 8")
        self.emu.cfg.update(freq=int(self.a.f0), ppm=self.ppm)

    def guarded(self, name, fn, *args):
        t = time.time()
        try:
            fn(*args)
            self.rec(test="phase", name=name, ok=True, secs=time.time() - t)
        except Exception as e:
            self.rec(test="phase", name=name, ok=False, secs=time.time() - t, error=repr(e), tb=traceback.format_exc()[-1500:])
            self.say("!! %s failed: %r" % (name, e))
            try: self.restart_emu()
            except Exception as e2: self.say("!! emulator restart failed: %r" % e2)

    # G: gain curve, 16-bit link (62.5 kS/s) so the 8-bit link scaling does not interfere
    def test_g(self):
        f0, rate, off = self.a.f0, 62500, 10e3
        self.vsg.set(f=f0 + off, p=-50, on=True)
        for g in range(0, 83, 2):
            self.emu.set(spec=0, rate=rate, freq=int(f0), gain=g, settle=0.15)
            ok, r, _ = self.iq(rate, secs=0.5, f_exp=off)
            if r: self.rec(test="G", gain=g, level=r["level"], floor_bin=r["floor_bin"], snr=r["snr"], peak=r["peak_abs"], n=r["n"])
        self.vsg.set(on=False)
        for g in range(0, 83, 6):
            self.emu.set(spec=0, rate=rate, freq=int(f0), gain=g, settle=0.15)
            ok, r, _ = self.iq(rate, secs=0.5)
            if r: self.rec(test="Goff", gain=g, floor_bin=r["floor_bin"], rms=r["rms"], n=r["n"])
        self.emu.set(gain=self.gain, settle=0.1)

    # L: level linearity down to the noise (MDS), IQ 62.5k (1 s captures, 1 Hz bins) and SPEC 80 MHz
    def test_l(self):
        f0 = self.a.f0
        for g in sorted(set([self.gain, 82])):
            self.emu.set(spec=0, rate=62500, freq=int(f0), gain=g, settle=0.2)
            for p in np.arange(-50, -131, -2):
                self.vsg.set(f=f0 + 10e3, p=p, on=True); time.sleep(0.1)
                ok, r, _ = self.iq(62500, secs=1.0, f_exp=10e3, search=300)
                if r: self.rec(test="L", mode="iq62k", gain=g, p=float(p), level=r["level"], floor_bin=r["floor_bin"], n=r["n"],
                               freq=r["freq"])
        for g in (40, 60):
            self.emu.set(spec=80000000, bins=1024, freq=int(f0), gain=g, dcfix=1, settle=0.3)
            for p in np.arange(-50, -111, -4):
                self.vsg.set(f=f0 + 10e6, p=p, on=True); time.sleep(0.1)
                self.emu.cap(3, self.tmp, timeout=3)
                ok, S, _ = self.spec(80e6, 1024, 40)
                if S is not None and len(S):
                    q = spec_peak(spec_mean(S), 80e6, 10e6, search_bins=2)
                    self.rec(test="L", mode="spec80", gain=g, p=float(p), level=q["level"], median=q["median"])
        self.vsg.set(p=-50, on=False)
        self.emu.set(spec=0, rate=250000, gain=self.gain, settle=0.1)

    # N: noise floor vs gain in SPEC (median of 80 MHz / 1024), VSG off
    def test_n(self):
        self.vsg.set(on=False)
        for g in range(0, 83, 6):
            self.emu.set(spec=80000000, bins=1024, freq=int(self.a.f0), gain=g, dcfix=1, settle=0.3)
            ok, S, _ = self.spec(80e6, 1024, 30)
            if S is not None and len(S):
                m = spec_mean(S)
                self.rec(test="N", gain=g, median=float(np.median(m)), p95=float(np.percentile(m, 95)), centre=float(m[512] - np.median(m)))
        # IQ noise density per rate (8-bit link at 250k, 16-bit below), dBFS/Hz, at the working gain and at 82
        for g in sorted(set([self.gain, 82])):
            for rate in (250000, 125000, 62500):
                self.emu.set(spec=0, rate=rate, freq=int(self.a.f0), gain=g, settle=0.2)
                ok, r, _ = self.iq(rate, secs=1.0)
                if r: self.rec(test="Niq", gain=g, rate=rate, floor_bin=r["floor_bin"], n=r["n"],
                               dbfs_hz=r["floor_bin"] - 10 * np.log10(1.5 * rate / r["n"]))
        self.emu.set(spec=0, rate=250000, gain=self.gain, settle=0.1)

    # R: spectra per second and integrity for every SPEC profile, mean and max hold
    def test_r(self):
        for fs in (16e6, 40e6, 80e6):
            for bins in (256, 1024, 2048):
                for mh in (0, 1):
                    self.emu.set(spec=int(fs), bins=bins, maxhold=mh, freq=int(self.a.f0), gain=self.gain, settle=0.5)
                    s0 = self.emu.stats(); t0 = time.time(); time.sleep(5); s1 = self.emu.stats(); dt = time.time() - t0
                    self.rec(test="R", fs=fs, bins=bins, maxhold=mh, rate=(s1["frames"] - s0["frames"]) / dt,
                             crc=s1["crc"] - s0["crc"], gaps=s1["gaps"] - s0["gaps"], lost=s1["lost"] - s0["lost"])
                    self.say("R %2d MHz/%4d %s: %.1f spectra/s" % (fs / MHZ, bins, "max " if mh else "mean", (s1["frames"] - s0["frames"]) / dt))
        self.emu.set(spec=0, rate=250000, maxhold=0, gain=self.gain, settle=0.1)

    # S: what the first samples after a retune look like (tone frequency / level in 2 ms slices)
    def test_s(self):
        f0, rate, off = self.a.f0, 250000, 40e3
        self.vsg.set(f=f0 + off, p=-50, on=True)
        for i in range(30):
            hop = (7e6, 1e6, 0.1e6, 30e6, 200e6)[i % 5]
            self.emu.set(spec=0, rate=rate, freq=int(f0 + hop), gain=self.gain, settle=0.25)
            self.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.gain, settle=0.0)
            ok, got, dt, _ = self.emu.cap(int(rate * 0.1), self.tmp, timeout=3)
            x = np.fromfile(self.tmp, np.complex64).astype(np.complex128)
            seg = 500   # 2 ms
            fr, lv = [], []
            for k in range(len(x) // seg):
                y = x[k * seg:(k + 1) * seg] * np.exp(-2j * np.pi * off * np.arange(k * seg, (k + 1) * seg) / rate)
                ph = np.unwrap(np.angle(y))
                fr.append(float(np.polyfit(np.arange(seg) / rate, ph, 1)[0] / (2 * np.pi)))
                lv.append(float(10 * np.log10(np.mean(np.abs(y) ** 2) + 1e-30)))
            self.rec(test="S", i=i, hop=hop, first_data_s=dt, freq_err=fr, level=lv)
        self.emu.set(freq=int(f0), settle=0.1)

    # F: phase noise of the received CW (VSG phase noise included), 62.5k and 250k, Welch PSD in dBc/Hz
    def test_f(self):
        f0 = self.a.f0
        for rate, off, secs in ((62500, 5e3, 4.0), (250000, 20e3, 2.0)):
            self.vsg.set(f=f0 + off, p=-50, on=True)
            self.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.gain, settle=0.3)
            ok, got, dt, _ = self.emu.cap(int(rate * secs), self.tmp, timeout=secs * 3)
            x = np.fromfile(self.tmp, np.complex64).astype(np.complex128)
            x = x * np.exp(-2j * np.pi * off * np.arange(len(x)) / rate)
            nseg = 8192 if rate < 1e5 else 16384
            w = np.hanning(nseg); P = None; k = 0
            for s in range(0, len(x) - nseg, nseg // 2):
                X = np.fft.fftshift(np.fft.fft(x[s:s + nseg] * w)); q = np.abs(X) ** 2
                P = q if P is None else P + q; k += 1
            P /= k
            f = (np.arange(nseg) - nseg // 2) * rate / nseg
            ctr = np.abs(f) < 3 * rate / nseg
            carrier = P[ctr].sum()
            enbw = 1.5 * rate / nseg
            L = 10 * np.log10(P / carrier / enbw + 1e-30)
            pts = {}
            for o in (100, 300, 1000, 3000, 10000, 30000, 100000):
                if o < rate * 0.45:
                    sel = (np.abs(np.abs(f) - o) < max(o * 0.1, 2 * rate / nseg))
                    pts[str(o)] = float(np.median(L[sel]))
            keep = slice(None, None, max(1, nseg // 2048))
            self.rec(test="F", rate=rate, offset=off, pn=pts, f=f[keep].tolist(), L=L[keep].tolist())
            self.say("F %d: %s" % (rate, ", ".join("%s Hz %.0f" % (k2, v) for k2, v in pts.items())))

    # W: long soak until the deadline: 20 min IQ / 20 min SPEC blocks with a check every minute, and after each
    # pair a mini LO sweep, 30 random retunes, a round of mode switches and 5 stop/start cycles
    def test_w(self, deadline, block=1200):
        f0 = self.a.f0; cyc = 0; rng = random.Random(7)
        iq_rates = (250000, 125000, 62500); specs = ((80e6, 1024), (40e6, 2048), (16e6, 256), (80e6, 256))
        while time.time() + 300 < deadline:
            for kind in ("iq", "spec"):
                if time.time() + 120 > deadline: break
                if kind == "iq":
                    rate = iq_rates[cyc % 3]; mode = (0, rate, 0, 0)
                else:
                    fs, bins = specs[cyc % 4]; mode = (fs, 0, bins, 0)
                self.apply_mode(mode, f0)
                s0 = self.emu.stats(); t0 = time.time(); end = min(deadline - 60, t0 + block)
                while time.time() < end:
                    time.sleep(max(0.0, min(60, end - time.time()) - 1.0))
                    s = self.emu.stats()
                    if kind == "iq":
                        off = 0.16 * rate
                        ok, r, _ = self.iq(rate, secs=0.5, f_exp=off, search=3000)
                        chk = dict(err_hz=r["freq"] - off, level=r["level"], cnr=r["level"] - r["floor_bin"], snr=r["snr"]) if r else {}
                    else:
                        self.emu.cap(2, self.tmp, timeout=3)
                        ok, S, _ = self.spec(fs, bins, 25)
                        if S is not None and len(S):
                            m = spec_mean(S); q = spec_peak(m, fs, 5e6, search_bins=3)
                            chk = dict(err_bins=q["err_bins"], level=q["level"], median=q["median"], mirror_db=q["mirror_db"],
                                       centre=float(m[bins // 2] - q["median"]))
                        else:
                            chk = {}
                    self.rec(test="W", cyc=cyc, kind=kind, mode=list(mode), el=round(time.time() - self.t_start, 1),
                             running=s["running"], **{k: s[k] - s0[k] for k in ("frames", "samples", "crc", "gaps", "lost")}, **chk)
                    if not s["running"]: raise RuntimeError("client stopped running")
                s1 = self.emu.stats()
                self.say("W cyc %d %s %s: %s" % (cyc, kind, mode, {k: s1[k] - s0[k] for k in ("frames", "crc", "gaps", "lost")}))
            if time.time() + 240 < deadline:
                self.mini(cyc, rng)
            cyc += 1

    def mini(self, cyc, rng):
        f0 = self.a.f0; fails = 0; n = 0
        for f in np.arange(2210e6, 2800e6 + 1, 40e6):
            self.vsg.set(f=f + 40e3, on=True)
            self.emu.set(spec=0, rate=250000, freq=int(f), gain=self.gain, settle=0.1)
            ok, r, _ = self.iq(250000, secs=0.26, f_exp=40e3, search=3000)
            self.rec(test="Wsweep", cyc=cyc, f=f, **({"err_hz": r["freq"] - 40e3, "level": r["level"]} if r else {}))
        for i in range(30):
            rate = rng.choice((250000, 125000, 62500)); off = 0.16 * rate
            f = rng.randrange(2215000, 2795000) * 1000
            self.vsg.set(f=f + off, on=True)
            self.emu.set(spec=0, rate=rate, freq=int(f), gain=self.gain, settle=0.02)
            good, r, _ = self.check_iq(rate, off); fails += not good; n += 1
        for m in self.MODES * 2:
            good, _, _ = self.apply_mode(m, f0); fails += not good; n += 1
        for i in range(5):
            self.emu.stop(); self.emu.start(); self.emu.cmd("sync 8")
            good, _, _ = self.apply_mode(self.MODES[i % len(self.MODES)], f0); fails += not good; n += 1
        self.rec(test="Wmini", cyc=cyc, n=n, fails=fails, **self.counters())
        self.say("W mini cyc %d: %d checks, %d failed, %s" % (cyc, n, fails, self.counters()))


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--emu", default=os.path.join(here, "..", "build", "Release", "esp_sdr_emu.exe"))
    ap.add_argument("--out", default=time.strftime("hwlong_%Y%m%d_%H%M"))
    ap.add_argument("--hours", type=float, default=6.0)
    ap.add_argument("--f0", type=float, default=2350e6)
    ap.add_argument("--gain", type=int, default=None)
    ap.add_argument("--soak", type=float, default=120)
    ap.add_argument("--tests", default="ABCDPGLNRSFW")
    a = ap.parse_args()
    r = Long(a)
    r.t_start = time.time(); deadline = r.t_start + a.hours * 3600
    try:
        r.emu.cmd("set spec=0 rate=250000 freq=%d gain=%d" % (a.f0, a.gain or 40))
        r.emu.start(); r.emu.cmd("sync 8")
        r.rec(test="start", args=vars(a), deadline=deadline)
        if a.gain is None: r.guarded("gain", r.pick_gain, a.f0)
        r.guarded("ppm", r.calibrate)
        T = a.tests
        if "A" in T: r.guarded("A", r.test_a)
        if "B" in T: r.guarded("ppm", r.calibrate); r.guarded("B", r.test_b)
        if "C" in T: r.guarded("ppm", r.calibrate); r.guarded("C", r.test_c)
        if "D" in T:
            r.guarded("ppm", r.calibrate)
            for name, fn in (("D1", r.test_d1), ("D2", r.test_d2), ("D3", r.test_d3)): r.guarded(name, fn)
            r.guarded("D4", r.test_d4, a.soak)
        if "P" in T: r.guarded("ppm", r.calibrate); r.guarded("P", r.test_p)
        for k, fn in (("G", r.test_g), ("L", r.test_l), ("N", r.test_n), ("R", r.test_r), ("S", r.test_s), ("F", r.test_f)):
            if k in T and time.time() + 600 < deadline:
                r.guarded("ppm", r.calibrate); r.guarded(k, fn)
        if "W" in T:
            r.guarded("ppm", r.calibrate)   # one calibration, then the drift over hours shows as frequency error
            r.rec(test="Wstart", ppm=r.ppm)
            while time.time() + 300 < deadline:
                r.guarded("W", r.test_w, deadline)
        r.rec(test="end", **r.counters())
        r.say("done: %s" % r.counters())
    finally:
        r.close()


if __name__ == "__main__":
    main()
