"""Quick look: CW from the VSG at f0+off, IQ at each rate/gain -> tone level, peak vs link FS, SNR, strongest spurs.
    python tools/hwprobe.py [--f0 2350e6] [--gains 0,10,20,30,40,50,60] [--p -50]"""
import argparse, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hwtest import VSG, Emu, iq_analyze, out_shift

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument("--port", default="COM4")
ap.add_argument("--emu", default=os.path.join(here, "..", "build", "Release", "esp_sdr_emu.exe"))
ap.add_argument("--f0", type=float, default=2350e6)
ap.add_argument("--p", type=float, default=-50)
ap.add_argument("--gains", default="0,10,20,30,40,50,60,70")
ap.add_argument("--ppm", type=float, default=1.8)
a = ap.parse_args()
tmp = os.path.join(os.environ.get("TEMP", "."), "probe.bin")
v = VSG(); e = Emu(a.emu, a.port)
try:
    e.cmd("set spec=0 rate=250000 freq=%d gain=30 ppm=%.3f" % (a.f0, a.ppm)); e.start(); e.cmd("sync 8")
    for rate, off in ((250000, 40e3), (62500, 10e3)):
        v.set(f=a.f0 + off, p=a.p, on=True)
        for g in [int(x) for x in a.gains.split(",")]:
            e.set(spec=0, rate=rate, freq=int(a.f0), gain=g, ppm="%.3f" % a.ppm, settle=0.3)
            ok, got, dt, _ = e.cap(int(rate * 0.5), tmp, timeout=5)
            r = iq_analyze(tmp, rate, off, 6000)
            fs8 = 2 ** (out_shift(g) + 7) / 32768.0
            print("rate %6d gain %2d: tone %+8.0f Hz %6.1f dBFS  peak %.4f (8b FS %.4f)  floor/bin %6.1f  snr %5.1f  img %5.1f  spurs %s" % (
                rate, g, r["freq"], r["level"], r["peak_abs"], fs8, r["floor_bin"], r["snr"], r["image_dbc"],
                " ".join("%+.0f:%.0f" % s for s in r["spurs"])), flush=True)
    v.set(on=False); e.stop()
finally:
    e.quit()
