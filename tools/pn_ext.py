"""Same phase-noise / NBFM tests as hwpn.py on a reference receiver, for a like-for-like comparison:
    python tools/pn_ext.py --dut hackrf --freqs 2350e6,2480e6,2700e6 [--tests PRF] [--gain auto]
    python tools/pn_ext.py --dut rtlsdr --freqs 1700e6                 (R820T2 tops out ~1.76 GHz)
Captures with hackrf_transfer (2 MS/s int8) / rtl_sdr (1 MS/s uint8), drops the start-up transient,
decimates with an FIR to the same 62.5k / 250k rates as the S3 and runs the hwpn.py analysis unchanged.
Use the same coax + attenuator chain for every DUT; VSG stays <= -50 dBm."""
import argparse, json, os, subprocess, sys, time
import numpy as np
from scipy import signal
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hwtest, hwpn

DUTS = {   # native rate, dropped start [s], gain ladder (high -> low), format
    "hackrf": dict(fs=2_000_000, drop=0.3, gains=[(32, 40), (24, 40), (24, 30), (16, 30), (16, 20), (8, 20), (8, 10), (0, 10)]),
    "rtlsdr": dict(fs=1_000_000, drop=0.3, gains=[49.6, 40.2, 33.8, 28.0, 22.9, 16.6, 12.5, 7.7, 2.7, 0.0]),
    # BB60C: gain = reference level [dBm]; 40 MS/s / 512 = 78.125k and / 128 = 312.5k, then x4/5 -> 62.5k / 250k
    "bb60c": dict(fs=None, drop=0.2, gains=[-30.0]),
}
BB_DLL = r"C:\Program Files\Signal Hound\Spike\bb_api.dll"


class BB60C:
    """Minimal bb_api.dll wrapper (IQ streaming, 32-bit complex floats). Spike must not hold the BB60C."""
    def __init__(self):
        import ctypes as C
        self.C = C; self.lib = C.CDLL(BB_DLL); self.dev = C.c_int(-1)
        self.ok(self.lib.bbOpenDevice(C.byref(self.dev)), "bbOpenDevice")

    def ok(self, st, what):
        if st < 0:
            self.lib.bbGetErrorString.restype = self.C.c_char_p
            raise RuntimeError("%s: %d %s" % (what, st, self.lib.bbGetErrorString(st).decode()))

    def capture(self, fc, rate, secs, ref):
        C, L, d = self.C, self.lib, self.dev.value
        ds = {62500: 512, 250000: 128}[rate]
        L.bbAbort(d)
        self.ok(L.bbConfigureRefLevel(d, C.c_double(ref)), "ref"); self.ok(L.bbConfigureGainAtten(d, -1, -1), "gain")
        self.ok(L.bbConfigureIQCenter(d, C.c_double(fc)), "center")
        self.ok(L.bbConfigureIQ(d, ds, C.c_double(0.8 * 40e6 / ds)), "iq")
        self.ok(L.bbConfigureIQDataType(d, 0), "dtype")                       # bbDataType32fc
        self.ok(L.bbInitiate(d, 4, 0), "initiate")                             # BB_STREAMING, BB_STREAM_IQ
        fs = 40e6 / ds; n = int(fs * (secs + 0.2)); blk = 16384
        out = np.empty(n, np.complex64); got = 0
        rem, loss, sec, nano = (C.c_int() for _ in range(4))
        while got < n:
            k = min(blk, n - got); buf = np.empty(k, np.complex64)
            self.ok(L.bbGetIQUnpacked(d, buf.ctypes.data_as(C.c_void_p), k, None, 0, 1 if got == 0 else 0,
                                      C.byref(rem), C.byref(loss), C.byref(sec), C.byref(nano)), "getiq")
            if loss.value and got > 0: raise RuntimeError("BB60C sample loss at %d / %d" % (got, n))
            out[got:got + k] = buf; got += k
        L.bbAbort(d)
        x = out[int(0.2 * fs):].astype(np.complex128)
        return signal.resample_poly(x, 4, 5)                                   # -> 62.5k / 250k

    def close(self):
        self.lib.bbAbort(self.dev.value); self.lib.bbCloseDevice(self.dev.value)


class ExtEmu:
    """Stands in for esp_sdr_emu: set() stores the tuning, cap() records n samples at the requested rate."""
    def __init__(self, a):
        self.a = a; self.cfg = {}; self.d = DUTS[a.dut]

    def set(self, wait=True, settle=0.0, **kw):
        self.cfg.update(kw); return 0.0

    # the rest of the esp_sdr_emu interface that hwcampaign.py calls
    def cmd(self, line): return "OK"
    def start(self): return 0.0
    def stop(self): pass
    def stats(self): return dict(crc=0, gaps=0, lost=0, running=1, tune="")
    def quit(self):
        if hasattr(self, "bb"): self.bb.close()

    def raw(self, fc, secs, gain):
        fs = self.d["fs"]; n = int(fs * (secs + self.d["drop"])); path = self.a.raw
        if self.a.dut == "hackrf":
            lna, vga = gain[0], gain[1]
            amp = gain[2] if len(gain) > 2 else 0          # optional 3rd element: front-end RF amp (+14 dB)
            cmd = ["hackrf_transfer", "-r", path, "-f", str(int(fc)), "-s", str(fs), "-n", str(n),
                   "-l", str(lna), "-g", str(vga), "-a", str(int(amp))]
        else:
            cmd = ["rtl_sdr", "-f", str(int(fc)), "-s", str(fs), "-g", str(gain), "-n", str(n), path]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=secs * 3 + 20, check=False)
        b = np.fromfile(path, np.int8 if self.a.dut == "hackrf" else np.uint8).astype(np.float64)
        if self.a.dut == "rtlsdr": b -= 127.4
        x = (b[0::2] + 1j * b[1::2]) / 128.0
        return x[int(fs * self.d["drop"]):]

    def cap(self, n, path, timeout=10):
        rate = int(self.cfg["rate"])
        if self.a.dut == "bb60c":
            if not hasattr(self, "bb"): self.bb = BB60C()
            y = self.bb.capture(self.cfg["freq"], rate, n / rate + 0.05, float(self.cfg["gain"]))[: n]
            y.astype(np.complex64).tofile(path)
            return True, len(y), 0.0, 0
        fs = self.d["fs"]; D = fs // rate
        x = self.raw(self.cfg["freq"], n / rate + 0.05, self.cfg["gain"])
        y = signal.resample_poly(x, 1, D)[: n]                  # FIR decimation to the S3 rate
        y.astype(np.complex64).tofile(path)
        return True, len(y), 0.0, 0


class ExtRun:
    """The parts of hwtest.Run / hwlong.Long that hwpn.PN uses."""
    def __init__(self, a):
        self.a = a
        os.makedirs(a.out, exist_ok=True)
        self.log = open(os.path.join(a.out, "results.jsonl"), "a", encoding="utf-8")
        self.tmp = os.path.join(a.out, "cap.bin"); a.raw = os.path.join(a.out, "raw.bin")
        self.vsg = hwtest.VSG() if not a.no_vsg else hwpn.NoVSG()
        self.emu = ExtEmu(a)
        self.gain = None

    rec = hwtest.Run.rec
    say = hwtest.Run.say

    def guarded(self, name, fn, *args):
        t = time.time()
        try:
            fn(*args); self.rec(test="phase", name=name, ok=True, secs=time.time() - t)
        except Exception as e:
            import traceback
            self.rec(test="phase", name=name, ok=False, error=repr(e), tb=traceback.format_exc()[-1500:])
            self.say("!! %s failed: %r" % (name, e))

    def pick_gain(self, f0):
        """Highest gain that keeps the -50 dBm CW below 0.7 FS (8-bit ADC) with >= 50 dB tone over the bin floor."""
        self.vsg.set(f=f0 + 40e3, p=-50, on=True)
        best = None
        for g in DUTS[self.a.dut]["gains"]:
            self.emu.set(spec=0, rate=250000, freq=int(f0), gain=g)
            x = self.emu.raw(f0, 0.3, g)
            pk = float(np.max(np.abs(np.concatenate([x.real, x.imag]))))
            self.emu.cap(int(250000 * 0.26), self.tmp)
            r = hwtest.iq_analyze(self.tmp, 250000, 40e3, 5e3)
            self.rec(test="gain", gain=g, peak_raw=pk, **{k: r[k] for k in ("level", "freq", "snr", "floor_bin")})
            self.say("gain %s: raw peak %.2f FS, tone %.1f dBFS, snr %.1f" % (g, pk, r["level"], r["snr"]))
            if pk < 0.7 and r["level"] - r["floor_bin"] > 50: best = g; break
        self.gain = best if best is not None else DUTS[self.a.dut]["gains"][-1]
        self.say("using gain %s" % (self.gain,))

    def calibrate(self): pass                      # TCXO references: the analysis locks onto the measured tone

    def counters(self): return {}

    def close(self):
        try: self.vsg.set(on=False)
        except Exception: pass
        if hasattr(self.emu, "bb"): self.emu.bb.close()
        self.log.close()


class ExtPN(hwpn.PN):
    def __init__(self, a):
        self.a = a; self.r = ExtRun(a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dut", choices=sorted(DUTS), required=True)
    ap.add_argument("--freqs", default=None)
    ap.add_argument("--gain", default="auto", help="auto, or hackrf 'lna,vga' / rtlsdr dB")
    ap.add_argument("--tests", default="PRF")
    ap.add_argument("--out", default=None)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--no-vsg", action="store_true")
    ap.add_argument("--atten", type=float, default=0.0, help="pad between VSG and DUT [dB], for the level axis")
    a = ap.parse_args()
    a.freqs = a.freqs or ("1700e6" if a.dut == "rtlsdr" else "2350e6,2480e6,2700e6")
    a.out = a.out or time.strftime("pn_%s_%%Y%%m%%d_%%H%%M" % a.dut)
    freqs = [float(v) for v in a.freqs.split(",")]
    t = ExtPN(a); r = t.r
    try:
        r.rec(test="start", dut=a.dut, args={k: v for k, v in vars(a).items()})
        if a.dut == "bb60c": r.gain = -30.0 if a.gain == "auto" else float(a.gain)
        elif a.gain == "auto" and not a.no_vsg: r.pick_gain(freqs[0])
        elif a.gain == "auto": r.gain = DUTS[a.dut]["gains"][len(DUTS[a.dut]["gains"]) // 2]
        else: r.gain = tuple(int(v) for v in a.gain.split(",")) if a.dut == "hackrf" else float(a.gain)
        for f0 in freqs:
            for k, fn in (("P", t.test_p), ("R", t.test_r), ("F", t.test_f)):
                if k in a.tests: r.guarded(k, fn, f0)
        r.rec(test="end")
    finally:
        try: t.fm_on(False)
        except Exception: pass
        r.close()
    hwpn.plots(a.out)


if __name__ == "__main__":
    main()
