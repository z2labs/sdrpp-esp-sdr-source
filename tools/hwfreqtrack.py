"""Instantaneous frequency of a VSG CW as seen by the S3 (62.5k IQ), to look at LO wander / periodic jumps.
    python tools/hwfreqtrack.py [--secs 10] [--gain 30] [--f0 2350e6] [--out freqtrack.png]"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hwtest import VSG, Emu
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument("--port", default="COM4")
ap.add_argument("--emu", default=os.path.join(here, "..", "build", "Release", "esp_sdr_emu.exe"))
ap.add_argument("--f0", type=float, default=2350e6)
ap.add_argument("--p", type=float, default=-50)
ap.add_argument("--gain", type=int, default=30)
ap.add_argument("--secs", type=float, default=10)
ap.add_argument("--out", default="freqtrack.png")
a = ap.parse_args()
rate, off = 62500, 10e3
tmp = os.path.join(os.environ.get("TEMP", "."), "ftrack.bin")
v = VSG(); e = Emu(a.emu, a.port)
try:
    e.cmd("set spec=0 rate=%d freq=%d gain=%d" % (rate, a.f0, a.gain)); e.start(); e.cmd("sync 8")
    v.set(f=a.f0 + off, p=a.p, on=True)
    e.set(spec=0, rate=rate, freq=int(a.f0), gain=a.gain, settle=0.5)
    s0 = e.stats()
    ok, got, dt, _ = e.cap(int(rate * a.secs), tmp, timeout=a.secs * 3 + 5)
    s1 = e.stats()
    v.set(on=False); e.stop()
finally:
    e.quit()
x = np.fromfile(tmp, np.complex64).astype(np.complex128)
# FFT peak per 50 ms segment (Hann, zero-padded, parabolic interpolation): no phase-unwrap ambiguity, no
# boxcar nulls (an earlier 2 ms block-average version aliased drift > +-250 Hz into fake jumps and fades)
seg = rate // 20; nfft = 1 << 16
w = np.hanning(seg); f = np.fft.fftfreq(nfft, 1 / rate)
win = np.abs(f - off) < 8000
t, fi, amp = [], [], []
for i in range(len(x) // seg):
    S = np.abs(np.fft.fft(x[i * seg:(i + 1) * seg] * w, nfft)) ** 2
    j = np.argmax(np.where(win, S, 0))
    a_, b_, c_ = np.log(S[j - 1] + 1e-30), np.log(S[j] + 1e-30), np.log(S[(j + 1) % nfft] + 1e-30)
    d = 0.5 * (a_ - c_) / (a_ - 2 * b_ + c_) if (a_ - 2 * b_ + c_) else 0.0
    t.append((i + 0.5) * seg / rate); fi.append((f[j] + d * rate / nfft) - off)
    amp.append(10 * np.log10(S[j] / (np.sum(w) ** 2) + 1e-30))
t, fi, amp = np.array(t), np.array(fi), np.array(amp)
k = seg
fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax[0].plot(t, fi, lw=0.6); ax[0].set_ylabel("carrier offset [Hz]"); ax[0].grid(alpha=0.3)
ax[1].plot(t, amp, lw=0.6); ax[1].set_ylabel("tone level [dBFS]"); ax[1].set_xlabel("time [s]"); ax[1].grid(alpha=0.3)
ax[0].set_title("S3 received CW at %.0f MHz, gain %d" % (a.f0 / 1e6, a.gain))
fig.tight_layout(); fig.savefig(a.out, dpi=120)
F = np.abs(np.fft.rfft((fi - fi.mean()) * np.hanning(len(fi)))); ff = np.fft.rfftfreq(len(fi), k / rate)
top = np.argsort(F[1:])[::-1][:6] + 1
slope = np.polyfit(t, fi, 1)[0]
print("mean offset %.1f Hz, p-p %.1f Hz, drift %.1f Hz/s (%.3f ppm/min); level %.1f dBFS p-p %.1f dB; gaps %d lost %d" % (
    fi.mean(), np.ptp(fi), slope, slope / a.f0 * 60e6, np.median(amp), np.ptp(amp),
    s1["gaps"] - s0["gaps"], s1["lost"] - s0["lost"]))
print("strongest FM rates [Hz]:", " ".join("%.2f" % ff[j] for j in top))
