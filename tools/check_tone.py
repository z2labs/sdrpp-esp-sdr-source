"""Hardware check of esp_sdr_test: VSG CW at f+OFF, capture, report tone/SNR/peaks (VSG on and off).
    python tools/check_tone.py [port] [freq_hz] [gain] [rate] [off_hz]"""
import os, socket, subprocess, sys, tempfile
import numpy as np

port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
F = float(sys.argv[2]) if len(sys.argv) > 2 else 2350e6
G = sys.argv[3] if len(sys.argv) > 3 else "60"
R = int(sys.argv[4]) if len(sys.argv) > 4 else 250000
OFF = float(sys.argv[5]) if len(sys.argv) > 5 else 40e3
EXE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build", "Release", "esp_sdr_test.exe")

v = socket.create_connection(("127.0.0.1", 5124), timeout=10); vf = v.makefile("rwb")
def vw(c): vf.write((c + "\n").encode()); vf.flush()
def vq(c): vw(c); return vf.readline().decode().strip()
vw(":OUTPut:MODulation:STATe OFF"); vw(":SOURce:POWer -50.00"); vw(":SOURce:FREQuency %d" % int(F + OFF))

def capture(on):
    vw(":OUTPut:STATe " + ("ON" if on else "OFF")); vq(":OUTPut:STATe?")
    fn = os.path.join(tempfile.gettempdir(), "esp_check.cf32")
    r = subprocess.run([EXE, port, "%d" % F, G, str(R), "4", fn], capture_output=True, text=True)
    res = [l for l in r.stdout.splitlines() if l.startswith("RESULT") or l.startswith("ERROR")]
    z = np.fromfile(fn, np.float32); z = z[0::2] + 1j * z[1::2]; z = z[R:]   # drop the first second
    nf = 8192; w = np.hanning(nf); k = len(z) // nf
    P = np.mean([np.abs(np.fft.fftshift(np.fft.fft(z[i * nf:(i + 1) * nf] * w))) ** 2 for i in range(k)], 0) / np.sum(w) ** 2
    f = np.fft.fftshift(np.fft.fftfreq(nf, 1 / R)); L = 10 * np.log10(P + 1e-30)
    fl = np.median(L[np.abs(f) < 0.4 * R])
    Q = L.copy(); peaks = []
    for _ in range(4):
        i = int(np.argmax(Q)); peaks.append((round(f[i] / 1e3, 2), round(L[i] - fl, 1))); Q[max(0, i - 15):i + 16] = -999
    sel = np.abs(f - OFF) < 5000; i = int(np.argmax(np.where(sel, L, -999)))
    tone = 10 * np.log10(P[i - 3:i + 4].sum() / 1.5)
    print(("VSG on " if on else "VSG off"), res[0] if res else r.stdout[-300:])
    print("   floor %.1f dBFS/bin, peaks (kHz, dB over floor): %s" % (fl, peaks))
    if on: print("   tone near +%.0f kHz: %+.3f kHz, %.1f dBFS, SNR %.1f dB" % (OFF / 1e3, f[i] / 1e3, tone, tone - fl))

capture(True); capture(False)
vw(":OUTPut:STATe OFF")
