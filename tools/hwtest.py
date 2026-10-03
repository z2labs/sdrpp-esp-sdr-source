"""Hardware test of the ESP-SDR client via esp_sdr_emu (scripted SDR++ stand-in) and a VSG (SCPI over TCP).
    python tools/hwtest.py [--port COM4] [--emu build/Release/esp_sdr_emu.exe] [--out DIR] [--tests ABCD] [--soak 600]
A: IQ CW sweep across the band (250k, 62.5k)      B: S3 LO sweep 2210-2800 MHz with the VSG following, VSG-off spur scan
C: SPEC 16/40/80 MHz bin accuracy / flatness, DC diagnostics (gain, bins, firmware DC 0/1)
D: stress - random retunes (+ drag bursts), IQ<->SPEC mode switches, start/stop cycles, soak
VSG power is clamped to <= -50 dBm. Results: results.jsonl in the output folder."""
import argparse, json, os, random, socket, subprocess, sys, time
import numpy as np

MHZ = 1e6


def out_shift(gain):   # same as Client::outShift
    gi = [0, 10, 20, 40, 50, 60, 70, 82]; gd = [0.0, 10.3, 21.1, 30.2, 41.9, 51.3, 61.3, 73.3]
    db = gd[7]
    for i in range(7):
        if gain <= gi[i + 1]:
            db = gd[i] + (gd[i + 1] - gd[i]) * (gain - gi[i]) / (gi[i + 1] - gi[i]); break
    return max(0, min(4, int(round(4 - (51.3 - db) / 6.02))))


class VSG:
    def __init__(self, host="127.0.0.1", port=5124):
        self.s = socket.create_connection((host, port), timeout=10)
        self.f = self.s.makefile("rwb")
        self.w(":OUTPut:MODulation:STATe OFF")
        self.set(p=-50)

    def w(self, c):
        self.f.write((c + "\n").encode()); self.f.flush()

    def q(self, c):
        self.w(c); return self.f.readline().decode().strip()

    def set(self, f=None, p=None, on=None):
        if p is not None: self.w(":SOURce:POWer %.2f" % min(float(p), -50.0))
        if f is not None: self.w(":SOURce:FREQuency %d" % int(round(f)))
        if on is not None: self.w(":OUTPut:STATe %s" % ("ON" if on else "OFF"))
        self.q(":SOURce:FREQuency?")   # sync: the VSG has applied everything before this answers
        time.sleep(0.03)


class Emu:
    def __init__(self, exe, port):
        self.p = subprocess.Popen([exe, port], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.cfg = {}

    def cmd(self, line):
        self.p.stdin.write(line + "\n"); self.p.stdin.flush()
        r = self.p.stdout.readline().strip()
        if not r: raise RuntimeError("esp_sdr_emu exited")
        return r

    def stats(self):
        r = self.cmd("stats")
        head, _, tune = r.partition(" tune=")
        d = {k: int(v) for k, v in (t.split("=") for t in head.split()[1:])}
        d["tune"] = tune
        return d

    def start(self):
        t = time.perf_counter(); r = self.cmd("start")
        if r != "OK": raise RuntimeError(r)
        return time.perf_counter() - t

    def stop(self): self.cmd("stop")

    def set(self, wait=True, settle=0.05, **kw):
        """Client::update(), like an SDR++ tune / rate change; optionally wait until the worker restarted the stream."""
        self.cfg.update(kw)
        t = time.perf_counter()
        self.cmd("set " + " ".join("%s=%s" % (k, v) for k, v in kw.items()))
        if wait:
            r = self.cmd("sync 8")
            if not r.startswith("SYNC"): raise RuntimeError("stream did not restart after set: " + r)
        dt = time.perf_counter() - t
        time.sleep(settle)
        return dt

    def cap(self, n, path, timeout=10):
        r = self.cmd("cap %d %s %g" % (n, path, timeout)).split()
        return r[0] == "CAP", int(r[1]), float(r[2]), int(r[3])

    def quit(self):
        try: self.cmd("quit")
        except Exception: pass
        self.p.wait(5)


def iq_analyze(path, fs, f_exp=None, search=None):
    """Tone at f_exp (Hz from centre, None: strongest): level dBFS, freq, SNR over fs, image, top spurs."""
    x = np.fromfile(path, np.complex64).astype(np.complex128)
    n = len(x)
    if n < 1024: return None
    w = np.hanning(n)
    P = np.abs(np.fft.fftshift(np.fft.fft(x * w))) ** 2 / (n * np.sum(w ** 2))   # sums to the mean power
    f = (np.arange(n) - n // 2) * fs / n
    dc = np.abs(f) < 3 * fs / n
    if f_exp is None:
        cand = np.where(~dc)[0]
    else:
        win = search if search else max(3000.0, 4 * fs / n)
        cand = np.where((np.abs(f - f_exp) <= win) & ~dc)[0]
        if len(cand) == 0: cand = np.where(~dc)[0]
    k = cand[np.argmax(P[cand])]
    a, b, c = (10 * np.log10(P[max(k - 1, 0):k + 2] + 1e-30) if 0 < k < n - 1 else (0, 1, 0))
    d = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) != 0 else 0.0
    ftone = (k + d - n // 2) * fs / n
    band = np.abs(np.arange(n) - k) <= 4
    tone = P[band].sum()
    noise_mask = ~band & ~dc
    noise = P[noise_mask].sum() * n / max(1, noise_mask.sum())          # whole band, tone/DC bins interpolated
    floor = np.median(P[noise_mask])
    km = n // 2 - (k - n // 2)                                           # mirror bin (-f)
    img = P[max(km - 4, 0):km + 5].sum() if 0 < km < n else 0
    spur_mask = noise_mask & (np.abs(np.arange(n) - km) > 4)
    Ls = 10 * np.log10(P + 1e-30); spurs = []
    m = spur_mask.copy()
    for _ in range(3):
        idx = np.where(m)[0]
        if len(idx) == 0: break
        j = idx[np.argmax(Ls[idx])]
        spurs.append((float(f[j]), float(Ls[j] - 10 * np.log10(tone + 1e-30))))
        m[max(j - 6, 0):j + 7] = False
    rms = np.sqrt(np.mean(np.abs(x) ** 2))
    return dict(level=10 * np.log10(tone + 1e-30), freq=float(ftone), snr=10 * np.log10(tone / max(noise, 1e-30)),
                floor_bin=10 * np.log10(floor + 1e-30), image_dbc=10 * np.log10((img + 1e-30) / (tone + 1e-30)),
                spurs=spurs, peak_abs=float(np.max(np.abs(x))), rms=float(rms), n=n)


def spec_load(path, bins):
    d = np.fromfile(path, np.float32)
    return d[: (len(d) // bins) * bins].reshape(-1, bins)


def spec_mean(S):
    return 10 * np.log10(np.mean(10 ** (S / 10), axis=0))


def spec_peak(m, fs, f_exp, search_bins=4, exclude_dc=2):
    n = len(m); df = fs / n
    j0 = int(round(n / 2 + f_exp / df))
    lo, hi = max(0, j0 - search_bins), min(n, j0 + search_bins + 1)
    idx = [j for j in range(lo, hi) if abs(j - n // 2) > exclude_dc]
    if not idx: return None
    j = max(idx, key=lambda q: m[q])
    jm = n - j if 0 < n - j < n else None
    return dict(freq=(j - n / 2) * df, err_bins=j - (n / 2 + f_exp / df), level=float(m[j]),
                median=float(np.median(m)), mirror_db=float(m[jm] - m[j]) if jm is not None else None,
                dc=float(m[n // 2] - np.median(m)))


class Run:
    def __init__(self, a):
        self.a = a
        os.makedirs(a.out, exist_ok=True)
        self.log = open(os.path.join(a.out, "results.jsonl"), "a", encoding="utf-8")
        self.tmp = os.path.join(a.out, "cap.bin")
        self.vsg = VSG()
        self.emu = Emu(a.emu, a.port)
        self.gain = a.gain
        self.ppm = 0.0

    def rec(self, **d):
        d["t"] = round(time.time(), 3)
        self.log.write(json.dumps(d) + "\n"); self.log.flush()

    def say(self, s):
        print(time.strftime("%H:%M:%S"), s, flush=True)

    def iq(self, rate, secs=0.26, f_exp=None, search=None):
        n = int(rate * secs)
        ok, got, dt, _ = self.emu.cap(n, self.tmp, timeout=max(3, secs * 4))
        r = iq_analyze(self.tmp, rate, f_exp, search) if got else None
        return ok, r, dt

    def spec(self, fs, bins, count=20):
        ok, got, dt, nb = self.emu.cap(count, self.tmp, timeout=5)
        S = spec_load(self.tmp, bins) if got else None
        return ok, S, dt

    def pick_gain(self, f0):
        """Highest gain (of a few) that keeps the CW at -50 dBm well below full scale in the 8-bit 250k link."""
        self.vsg.set(f=f0 + 40e3, on=True)
        best = None
        for g in (70, 60, 50, 40, 30, 20):
            self.emu.set(spec=0, rate=250000, freq=int(f0), gain=g, settle=0.2)
            ok, r, _ = self.iq(250000)
            if not r: continue
            fs8 = 2 ** (out_shift(g) + 7) / 32768.0          # full scale of the 8-bit link at this gain
            self.rec(test="gain", gain=g, fs8=fs8, **{k: r[k] for k in ("level", "freq", "snr", "peak_abs")})
            self.say("gain %d: tone %+.0f Hz level %.1f dBFS peak %.4f (8-bit FS %.4f) snr %.1f" % (
                g, r["freq"], r["level"], r["peak_abs"], fs8, r["snr"]))
            if r["peak_abs"] < 0.75 * fs8 and r["level"] - r["floor_bin"] > 50:   # |I+jQ| of a tone: ~6 dB headroom per axis
                best = g; break
        self.gain = best if best is not None else 30
        self.say("using gain %d" % self.gain)

    def calibrate(self):
        """PPM like the SDR++ slider: CW at f0 + 40 kHz, measure, correct, verify."""
        f0 = self.a.f0
        self.vsg.set(f=f0 + 40e3, on=True)
        for it in range(3):
            self.emu.set(spec=0, rate=250000, freq=int(f0), gain=self.gain, ppm="%.3f" % self.ppm, settle=0.2)
            ok, r, _ = self.iq(250000, secs=0.5, f_exp=40e3, search=30e3)
            err = r["freq"] - 40e3
            self.rec(test="ppm", it=it, ppm=self.ppm, err_hz=err, level=r["level"])
            self.say("ppm %.3f: tone error %+.1f Hz" % (self.ppm, err))
            if abs(err) < 20: break
            self.ppm += -err / f0 * 1e6
        self.emu.cfg["ppm"] = self.ppm

    # ---------------- A: IQ CW sweep across the output band ----------------
    def test_a(self):
        f0 = self.a.f0
        for rate in (250000, 62500):
            self.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.gain, settle=0.3)
            offs = sorted(set([round(x) for x in np.linspace(-0.48 * rate, 0.48 * rate, 25)] +
                              [rate * s for s in (-0.02, -0.008, 0.008, 0.02)]))
            for off in offs:
                if abs(off) < 1: continue
                self.vsg.set(f=f0 + off, on=True)
                time.sleep(0.05)
                ok, r, _ = self.iq(rate, f_exp=off)
                if not r: self.rec(test="A", rate=rate, off=off, ok=False); continue
                err = r["freq"] - off
                self.rec(test="A", rate=rate, off=off, ok=ok, err_hz=err, **{k: r[k] for k in ("level", "snr", "image_dbc", "spurs", "floor_bin")})
                self.say("A %6d  off %+8.0f  err %+7.1f Hz  level %6.1f  snr %5.1f  image %6.1f dBc" % (rate, off, err, r["level"], r["snr"], r["image_dbc"]))

    # ---------------- B: S3 LO sweep with the VSG following, then VSG-off spur scan ----------------
    def test_b(self):
        rate = 250000
        for f in np.arange(2210e6, 2800e6 + 1, 10e6):
            self.vsg.set(f=f + 40e3, on=True)
            self.emu.set(spec=0, rate=rate, freq=int(f), gain=self.gain, settle=0.15)
            ok, r, _ = self.iq(rate, f_exp=40e3, search=20e3)
            if not r: self.rec(test="B", f=f, ok=False); continue
            err = r["freq"] - 40e3
            self.rec(test="B", f=f, ok=ok, err_hz=err, ppm=err / f * 1e6, **{k: r[k] for k in ("level", "snr", "image_dbc", "spurs", "floor_bin")})
            self.say("B %7.1f MHz  err %+7.1f Hz (%+.2f ppm)  level %6.1f  snr %5.1f" % (f / MHZ, err, err / f * 1e6, r["level"], r["snr"]))
        self.vsg.set(on=False)
        for f in np.arange(2210e6, 2800e6 + 1, 2e6):
            self.emu.set(spec=0, rate=rate, freq=int(f), gain=self.gain, settle=0.12)
            ok, r, _ = self.iq(rate)
            if not r: continue
            sp = r["level"] - r["floor_bin"]      # strongest line above the per-bin floor
            self.rec(test="Boff", f=f, ok=ok, peak_freq=r["freq"], peak_level=r["level"], floor_bin=r["floor_bin"], above=sp)
            if sp > 15: self.say("Boff %7.1f MHz  line at %+8.0f Hz  %6.1f dBFS (%.1f dB above floor)" % (f / MHZ, r["freq"], r["level"], sp))

    # ---------------- C: on-chip spectrum (SPEC) accuracy and DC diagnostics ----------------
    def test_c(self):
        f0 = self.a.f0
        npz = {}
        for fs, bins, step in ((80e6, 1024, 2e6), (40e6, 1024, 1e6), (16e6, 256, 0.4e6), (80e6, 2048, 4e6)):
            self.emu.set(spec=int(fs), bins=bins, freq=int(f0), gain=self.gain, maxhold=0, dcfix=1, settle=0.3)
            for k in range(-19, 20):
                off = k * step if bins != 2048 else k * step / 2
                self.vsg.set(f=f0 + off, on=True)
                time.sleep(0.05)
                self.emu.cap(3, self.tmp, timeout=3)          # drop spectra that straddle the VSG retune
                ok, S, _ = self.spec(fs, bins, 15)
                if S is None or not len(S): self.rec(test="C", fs=fs, bins=bins, off=off, ok=False); continue
                m = spec_mean(S)
                p = spec_peak(m, fs, off, exclude_dc=-1 if k == 0 else 2)
                self.rec(test="C", fs=fs, bins=bins, off=off, ok=ok, n=len(S), **p)
                self.say("C %2d MHz/%4d off %+6.1f MHz  bin err %+.2f  level %6.1f  median %6.1f  mirror %s" % (
                    fs / MHZ, bins, off / MHZ, p["err_bins"], p["level"], p["median"],
                    "%.1f" % p["mirror_db"] if p["mirror_db"] is not None else "-"))
        # DC hump vs gain, bins and the firmware DC tracker, VSG off
        self.vsg.set(on=False)
        for dcm in (1, 0):   # 1: firmware default DC 1 (averaged), 0: module fix (DC 0 + centre bin filled)
            for g in (5, 28, 50):
                for bins in (256, 1024, 2048):
                    self.emu.set(spec=80000000, bins=bins, freq=int(f0), gain=g, dcfix=0 if dcm else 1, settle=0.4)
                    ok, S, _ = self.spec(80e6, bins, 60)
                    if S is None or not len(S): continue
                    m = spec_mean(S); med = np.median(m); n = bins
                    f = (np.arange(n) - n / 2) * 80e6 / n
                    hump = f[(m > med + 6) & (np.abs(f) < 15e6)]
                    width = (hump.max() - hump.min()) if len(hump) else 0.0
                    npz["dc%d_g%d_n%d" % (dcm, g, bins)] = m
                    self.rec(test="Cdc", dc_mode=dcm, gain=g, bins=bins, ok=ok, dc=float(m[n // 2] - med),
                             hump_mhz=width / MHZ, median=float(med), max_db=float(m.max()), max_freq=float(f[np.argmax(m)]))
                    self.say("Cdc %s gain %2d bins %4d: centre %+5.1f dB over median, hump %.2f MHz wide, median %.1f" % (
                        "DC1" if dcm else "fix", g, bins, m[n // 2] - med, width / MHZ, med))
        self.emu.set(dcfix=1, gain=self.gain, settle=0.2)
        np.savez(os.path.join(self.a.out, "spec_dc.npz"), **npz)

    # ---------------- D: stress ----------------
    def check_iq(self, rate, off):
        ok, r, dt = self.iq(rate, secs=0.12, f_exp=off, search=max(3000.0, 0.3 * off))
        good = bool(ok and r and abs(r["freq"] - off) < 1500 + 5e-6 * self.emu.cfg.get("freq", 2.4e9)
                    and r["level"] - r["floor_bin"] > 25)
        return good, r, dt

    def check_spec(self, fs, bins, off):
        self.emu.cap(2, self.tmp, timeout=3)
        ok, S, dt = self.spec(fs, bins, 5)
        if S is None or not len(S): return False, None, dt
        p = spec_peak(spec_mean(S), fs, off, search_bins=3)
        good = bool(ok and p and abs(p["err_bins"]) <= 1.5 and p["level"] - p["median"] > 15)
        return good, p, dt

    def counters(self):
        s = self.emu.stats(); return {k: s[k] for k in ("crc", "gaps", "lost", "running")}

    def test_d1(self, n=300):
        """Random retunes over the whole range, random rate; every 10th is a 'drag' burst of 20 quick updates."""
        rng = random.Random(1); fails = 0; c0 = self.counters()
        for i in range(n):
            rate = rng.choice((250000, 125000, 62500)); off = 0.16 * rate
            f = rng.randrange(2215000, 2795000) * 1000
            self.vsg.set(f=f + off, on=True)
            if i % 10 == 9:
                for j in range(20):
                    self.emu.set(wait=False, settle=0.01, spec=0, rate=rate, freq=int(f - (19 - j) * 25000), gain=self.gain)
            lat = self.emu.set(spec=0, rate=rate, freq=int(f), gain=self.gain, settle=0.02)
            good, r, dt = self.check_iq(rate, off)
            fails += not good
            self.rec(test="D1", i=i, f=f, rate=rate, drag=i % 10 == 9, good=good, retune_s=lat, cap_s=dt,
                     **({"err_hz": r["freq"] - off, "level": r["level"]} if r else {}))
            if not good: self.say("D1 %d FAIL f %.3f MHz rate %d: %s" % (i, f / MHZ, rate, r and (r["freq"], r["level"])))
        c1 = self.counters()
        self.rec(test="D1sum", n=n, fails=fails, **{k: c1[k] - c0[k] for k in ("crc", "gaps", "lost")})
        self.say("D1 %d retunes, %d failed, counters %s -> %s" % (n, fails, c0, c1))

    MODES = [(0, 250000, 0, 0), (80e6, 0, 256, 0), (0, 62500, 0, 0), (40e6, 0, 1024, 0), (0, 125000, 0, 0),
             (16e6, 0, 2048, 0), (80e6, 0, 2048, 1)]

    def apply_mode(self, m, f0):
        fs, rate, bins, mh = m
        if fs:
            off = 5e6; self.vsg.set(f=f0 + off, on=True)
            lat = self.emu.set(spec=int(fs), bins=bins, maxhold=mh, freq=int(f0), gain=self.gain, settle=0.1)
            good, p, dt = self.check_spec(fs, bins, off)
        else:
            off = 0.16 * rate; self.vsg.set(f=f0 + off, on=True)
            lat = self.emu.set(spec=0, rate=rate, freq=int(f0), gain=self.gain, settle=0.05)
            good, p, dt = self.check_iq(rate, off)
        return good, lat, dt

    def test_d2(self, n=100):
        f0 = self.a.f0; fails = 0; c0 = self.counters()
        for i in range(n):
            m = self.MODES[i % len(self.MODES)]
            good, lat, dt = self.apply_mode(m, f0)
            fails += not good
            self.rec(test="D2", i=i, mode=list(m), good=good, switch_s=lat, first_data_s=dt)
            if not good: self.say("D2 %d FAIL mode %s" % (i, m))
        c1 = self.counters()
        self.rec(test="D2sum", n=n, fails=fails, **{k: c1[k] - c0[k] for k in ("crc", "gaps", "lost")})
        self.say("D2 %d mode switches, %d failed, counters %s -> %s" % (n, fails, c0, c1))

    def test_d3(self, n=50):
        """Play / stop like in SDR++: stop the client, start it again in a random mode, check the data."""
        f0 = self.a.f0; rng = random.Random(3); fails = 0
        for i in range(n):
            self.emu.stop()
            m = rng.choice(self.MODES)
            fs, rate, bins, mh = m
            self.emu.cmd("set spec=%d rate=%d bins=%d maxhold=%d freq=%d gain=%d" % (fs, rate or 250000, bins or 256, mh, f0, self.gain))
            t = time.perf_counter()
            self.emu.start()
            r = self.emu.cmd("sync 8")
            start_s = time.perf_counter() - t
            good, lat, dt = self.apply_mode(m, f0)
            fails += not good
            self.rec(test="D3", i=i, mode=list(m), good=good, start_s=start_s, first_data_s=dt, sync=r)
            if not good: self.say("D3 %d FAIL mode %s" % (i, m))
        self.rec(test="D3sum", n=n, fails=fails, **self.counters())
        self.say("D3 %d start/stop cycles, %d failed" % (n, fails))

    def test_d4(self, secs):
        """Soak: SPEC 80 MHz / 256 bins, then IQ 250 kS/s; counters every 10 s, signal check every 60 s."""
        f0 = self.a.f0
        for m in ((80e6, 0, 256, 0), (0, 250000, 0, 0)):
            good, _, _ = self.apply_mode(m, f0)
            c0 = self.emu.stats(); t0 = time.time(); last = c0; checks = fails = 0
            while time.time() - t0 < secs:
                time.sleep(10)
                s = self.emu.stats(); el = time.time() - t0
                if int(el) // 60 != int(el - 10) // 60:
                    g, _, _ = self.check_spec(m[0], m[2], 5e6) if m[0] else self.check_iq(m[1], 0.16 * m[1])
                    checks += 1; fails += not g
                self.rec(test="D4", mode=list(m), el=round(el, 1), **{k: s[k] - c0[k] for k in ("frames", "samples", "crc", "gaps", "lost")},
                         running=s["running"], rate=(s["samples"] - last["samples"]) / 10.0)
                last = s
            s = self.emu.stats(); el = time.time() - t0
            d = {k: s[k] - c0[k] for k in ("frames", "samples", "crc", "gaps", "lost")}
            self.rec(test="D4sum", mode=list(m), secs=el, checks=checks, fails=fails, running=s["running"],
                     mean_rate=d["samples"] / el, **d)
            self.say("D4 %s %.0f s: %s, %.1f /s, signal checks %d failed %d" % (m, el, d, d["samples"] / el, checks, fails))

    # ---------------- P: PLL frequency error map (drift-corrected) ----------------
    def test_p(self):
        """Tone error vs tuned frequency on a fine grid; a reference at f0 every few points removes the crystal drift."""
        rate, off, f0 = 250000, 40e3, self.a.f0
        grid = list(np.arange(2400e6, 2460e6 + 1, 1e6)) + list(np.arange(2430e6, 2431e6, 50e3)) + \
               list(np.arange(2210e6, 2800e6 + 1, 5e6))

        def meas(f):
            self.vsg.set(f=f + off, on=True)
            self.emu.set(spec=0, rate=rate, freq=int(f), gain=self.gain, settle=0.1)
            ok, r, _ = self.iq(rate, secs=0.4, f_exp=off, search=3000)
            return r["freq"] - off if r else np.nan

        ref_t, ref_e = [], []
        for i, f in enumerate(grid):
            if i % 6 == 0:
                ref_e.append(meas(f0)); ref_t.append(time.time())
            e = meas(f); t = time.time()
            self.rec(test="P", f=f, err_hz=e, ref_err_hz=float(np.interp(t, ref_t, ref_e)), tref=ref_t[-1])
        ref_e.append(meas(f0)); ref_t.append(time.time())
        self.rec(test="Pref", t=ref_t, err=ref_e)
        self.say("P done: %d points, reference drift %+.0f Hz" % (len(grid), ref_e[-1] - ref_e[0]))

    def close(self):
        try: self.vsg.set(on=False)
        except Exception: pass
        self.emu.quit()


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--emu", default=os.path.join(here, "..", "build", "Release", "esp_sdr_emu.exe"))
    ap.add_argument("--out", default=time.strftime("hwtest_%Y%m%d_%H%M"))
    ap.add_argument("--tests", default="ABCD")
    ap.add_argument("--f0", type=float, default=2350e6)
    ap.add_argument("--gain", type=int, default=None)
    ap.add_argument("--soak", type=float, default=600)
    a = ap.parse_args()
    r = Run(a)
    try:
        r.emu.cmd("set spec=0 rate=250000 freq=%d gain=%d" % (a.f0, a.gain or 40))
        r.emu.start(); r.emu.cmd("sync 8")
        r.rec(test="start", args=vars(a))
        if a.gain is None: r.pick_gain(a.f0)
        r.calibrate()
        if "A" in a.tests: r.test_a()
        if "B" in a.tests: r.calibrate(); r.test_b()
        if "C" in a.tests: r.calibrate(); r.test_c()
        if "D" in a.tests:
            r.calibrate(); r.test_d1(); r.test_d2(); r.test_d3(); r.test_d4(a.soak)
        if "P" in a.tests: r.calibrate(); r.test_p()
        r.rec(test="end", **r.counters())
        r.say("done: %s" % r.counters())
    finally:
        r.close()


if __name__ == "__main__":
    main()
