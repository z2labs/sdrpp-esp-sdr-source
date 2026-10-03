"""Plots and a Markdown summary from a hwtest.py run.   python tools/hwreport.py RUN_DIR"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

d = sys.argv[1]
R = [json.loads(l) for l in open(os.path.join(d, "results.jsonl"), encoding="utf-8") if l.strip()]
T = lambda name: [r for r in R if r.get("test") == name]
out = []
say = out.append


def fig(name):
    p = os.path.join(d, name); plt.tight_layout(); plt.savefig(p, dpi=110); plt.close(); return name


# ---- A ----
A = T("A")
if A:
    f, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=False)
    say("## A. IQ CW sweep across the band (LO fixed)\n")
    for rate, c in ((250000, "C0"), (62500, "C1")):
        a = sorted([r for r in A if r["rate"] == rate and "level" in r], key=lambda r: r["off"])
        if not a: continue
        x = np.array([r["off"] for r in a]) / 1e3
        lv = np.array([r["level"] for r in a]); ref = np.median(lv)
        ax[0].plot(x, lv - ref, "o-", c=c, label="%g kS/s" % (rate / 1e3))
        ax[1].plot(x, [r["err_hz"] for r in a], "o-", c=c)
        ax[2].plot(x, [r["image_dbc"] for r in a], "o-", c=c)
        inner = np.abs(x) <= 0.32 * rate / 1e3
        say("- %g kS/s: level %.1f dBFS, flatness within +-%.0f kHz: %.2f dB p-p, edge (+-%.0f kHz) %.1f dB; image %.1f dBc worst; "
            "integrated SNR (incl. phase noise) %.1f dB median" % (rate / 1e3, ref, 0.32 * rate / 1e3, np.ptp(lv[inner]),
            0.48 * rate / 1e3, (lv - ref)[np.argmax(np.abs(x))], max(r["image_dbc"] for r in a), np.median([r["snr"] for r in a])))
    ax[0].set_ylabel("level rel. median [dB]"); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].set_ylabel("freq. error [Hz]"); ax[1].grid(alpha=.3)
    ax[2].set_ylabel("image [dBc]"); ax[2].set_xlabel("tone offset [kHz]"); ax[2].grid(alpha=.3)
    say("\n![A](%s)\n" % fig("A_iq_sweep.png"))

# ---- B ----
B = [r for r in T("B") if "level" in r]
if B:
    say("## B. S3 LO sweep 2210-2800 MHz, VSG following at +40 kHz\n")
    fr = np.array([r["f"] for r in B]) / 1e6
    f, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    ax[0].plot(fr, [r["level"] for r in B], "o-"); ax[0].set_ylabel("level [dBFS]")
    ax[1].plot(fr, [r["err_hz"] for r in B], "o-"); ax[1].set_ylabel("freq. error [Hz]")
    ax[2].plot(fr, [r["level"] - r["floor_bin"] for r in B], "o-"); ax[2].set_ylabel("tone / bin floor [dB]")
    off = [r for r in T("Boff")]
    if off:
        ax2 = ax[2].twinx()
        ax2.plot([r["f"] / 1e6 for r in off], [r["above"] for r in off], ".", c="C3", ms=4)
        ax2.set_ylabel("VSG off: strongest line over floor [dB]", color="C3")
    for a in ax: a.grid(alpha=.3)
    ax[2].set_xlabel("frequency [MHz]")
    e = np.array([r["err_hz"] for r in B]); lv = np.array([r["level"] for r in B])
    say("- level %.1f ... %.1f dBFS over the range (VSG -50 dBm, near-field coupling, not a calibrated gain)" % (lv.min(), lv.max()))
    say("- frequency error after one PPM calibration at 2350 MHz: %.0f ... %+.0f Hz (std %.0f Hz)" % (e.min(), e.max(), e.std()))
    if off:
        spurs = [r for r in off if r["above"] > 15]
        say("- VSG off, 2 MHz steps: %d of %d tunings show a line > 15 dB above the bin floor%s" % (
            len(spurs), len(off), (": " + ", ".join("%.0f MHz (%+.1f kHz, %.0f dB)" % (r["f"] / 1e6, r["peak_freq"] / 1e3, r["above"]) for r in spurs[:12])) if spurs else ""))
    say("\n![B](%s)\n" % fig("B_lo_sweep.png"))

# ---- C ----
C = [r for r in T("C") if "level" in r]
if C:
    say("## C. On-chip spectrum (SPEC): bin accuracy, flatness, mirror\n")
    cfgs = sorted(set((r["fs"], r["bins"]) for r in C))
    f, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for fs, bins in cfgs:
        c = sorted([r for r in C if r["fs"] == fs and r["bins"] == bins], key=lambda r: r["off"])
        x = np.array([r["off"] for r in c]) / fs   # fraction of span
        lv = np.array([r["level"] for r in c])
        lab = "%g MHz / %d" % (fs / 1e6, bins)
        ax[0].plot(x, [r["err_bins"] for r in c], "o-", label=lab)
        ax[1].plot(x, lv - np.median(lv), "o-")
        ax[2].plot(x, [r["mirror_db"] if r["mirror_db"] is not None else np.nan for r in c], "o-")
        nz = [r for r in c if abs(r["off"]) > 0]
        say("- %s: bin error max %.2f bins; level p-p over +-40%% span %.1f dB; mirror worst %.1f dB; median %.1f dBFS" % (
            lab, max(abs(r["err_bins"]) for r in nz), np.ptp([r["level"] for r in nz if abs(r["off"]) <= 0.4 * fs]),
            max(r["mirror_db"] for r in nz if r["mirror_db"] is not None), np.median([r["median"] for r in c])))
    ax[0].set_ylabel("peak bin error [bins]"); ax[0].legend(fontsize=8)
    ax[1].set_ylabel("level rel. median [dB]"); ax[2].set_ylabel("mirror rel. tone [dB]")
    ax[2].set_xlabel("tone offset [fraction of span]")
    for a in ax: a.grid(alpha=.3)
    say("\n![C](%s)\n" % fig("C_spec.png"))

Cdc = T("Cdc")
if Cdc:
    say("### Centre (DC / LO leakage) in SPEC 80 MHz, VSG off\n")
    say("| DC handling | gain | bins | centre over median [dB] | hump > +6 dB width [MHz] | median [dBFS] |")
    say("| --- | --- | --- | --- | --- | --- |")
    for r in Cdc:
        say("| %s | %d | %d | %+.1f | %.2f | %.1f |" % ("firmware DC 1 (averaged)" if r["dc_mode"] else "DC 0 + centre filled", r["gain"], r["bins"], r["dc"], r["hump_mhz"], r["median"]))
    npz = os.path.join(d, "spec_dc.npz")
    if os.path.exists(npz):
        z = np.load(npz)
        f, ax = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
        for i, dcm in enumerate((1, 0)):
            for g in (5, 28, 50):
                k = "dc%d_g%d_n1024" % (dcm, g)
                if k not in z: continue
                m = z[k]; n = len(m); fx = (np.arange(n) - n / 2) * 80 / n
                ax[i].plot(fx, m - np.median(m), lw=.8, label="gain %d" % g)
            ax[i].set_xlim(-15, 15); ax[i].grid(alpha=.3); ax[i].set_xlabel("offset from centre [MHz]")
            ax[i].set_title("SPEC 80 MHz / 1024, %s" % ("firmware DC 1 (averaged)" if dcm else "DC 0 + centre filled")); ax[i].legend()
        ax[0].set_ylabel("dB over median")
        say("\n![Cdc](%s)\n" % fig("C_dc.png"))

# ---- D ----
say("## D. Stress (SDR++ host calls via esp_sdr_emu)\n")
for name, label in (("D1sum", "random retunes (all rates, every 10th a 20-step drag burst)"), ("D2sum", "IQ <-> SPEC mode switches"),
                    ("D3sum", "stop / start cycles")):
    for r in T(name):
        say("- %s: %d, failed %d, CRC %d, gaps %d, lost %d" % (label, r["n"], r["fails"], r.get("crc", 0), r.get("gaps", 0), r.get("lost", 0)))
D1 = T("D1")
if D1:
    lat = np.array([r["retune_s"] for r in D1]) * 1e3
    say("- retune latency (update -> stream running): median %.0f ms, 95%% %.0f ms, max %.0f ms" % (np.median(lat), np.percentile(lat, 95), lat.max()))
for name, key, label in (("D2", "switch_s", "mode switch"), ("D3", "start_s", "start (port open + CAPS + stream)")):
    rows = T(name)
    if rows:
        v = np.array([r[key] for r in rows]) * 1e3
        say("- %s latency: median %.0f ms, max %.0f ms" % (label, np.median(v), v.max()))
for r in T("D4sum"):
    say("- soak %s %.0f s: frames %d, CRC %d, gaps %d, lost %d, %.1f %s/s, signal checks %d failed %d" % (
        "SPEC %g MHz / %d" % (r["mode"][0] / 1e6, r["mode"][2]) if r["mode"][0] else "IQ %g kS/s" % (r["mode"][1] / 1e3),
        r["secs"], r["frames"], r["crc"], r["gaps"], r["lost"], r["mean_rate"], "spectra" if r["mode"][0] else "samples",
        r["checks"], r["fails"]))
if D1:
    f, ax = plt.subplots(1, 2, figsize=(11, 3.5))
    ax[0].hist(lat, bins=40); ax[0].set_xlabel("retune latency [ms]"); ax[0].set_ylabel("count"); ax[0].grid(alpha=.3)
    e = [r.get("err_hz", np.nan) for r in D1]
    ax[1].plot([r["f"] / 1e6 for r in D1], e, ".", ms=4); ax[1].set_xlabel("frequency [MHz]"); ax[1].set_ylabel("tone error [Hz]")
    ax[1].grid(alpha=.3)
    say("\n![D](%s)\n" % fig("D_stress.png"))

P = [r for r in T("P") if r["err_hz"] == r["err_hz"]]
if P:
    f0 = T("start")[0]["args"]["f0"]
    fr = np.array([r["f"] for r in P]); e = np.array([r["err_hz"] - r["ref_err_hz"] * r["f"] / f0 for r in P])
    say("## P. Tuning error map (crystal drift removed with a reference at %.0f MHz)\n" % (f0 / 1e6))
    f, ax = plt.subplots(1, 2, figsize=(12, 4))
    ax[0].plot(fr / 1e6, e, ".", ms=5); ax[0].set_xlabel("tuned frequency [MHz]"); ax[0].set_ylabel("tone error [Hz]")
    ax[1].plot((fr % 30e6) / 1e6, e, ".", ms=5); ax[1].set_xlabel("tuned frequency mod 30 MHz [MHz]")
    for a in ax: a.grid(alpha=.3)
    say("- error after drift removal: %.0f ... %+.0f Hz, std %.0f Hz over %d points" % (e.min(), e.max(), e.std(), len(e)))
    fine = [(r["f"], x) for r, x in zip(P, e) if 2430e6 <= r["f"] < 2431e6]
    if fine: say("- within 2430-2431 MHz (50 kHz steps): %.0f ... %+.0f Hz" % (min(x for _, x in fine), max(x for _, x in fine)))
    np.savetxt(os.path.join(d, "P_error_map.csv"), np.c_[fr, e], delimiter=",", header="f_hz,err_hz", comments="")
    say("\n![P](%s)\n" % fig("P_tuning_error.png"))

st = T("start"); en = T("end")
hdr = ["# ESP-SDR SDR++ module: hardware test", ""]
if st: hdr.append("Run %s, port %s, f0 %.0f MHz. VSG CW -50 dBm, near-field coupling." % (
    os.path.basename(os.path.abspath(d)), st[0]["args"]["port"], st[0]["args"]["f0"] / 1e6))
g = T("gain"); p = T("ppm")
if p: hdr.append("Gain index %s, PPM calibration %.3f ppm (residual %+.1f Hz)." % (
    "auto" if not g else "%d" % [r["gain"] for r in g][-1], p[-1]["ppm"], p[-1]["err_hz"]))
if en: hdr.append("Total CRC errors %d, gaps %d, lost samples %d." % (en[0]["crc"], en[0]["gaps"], en[0]["lost"]))
open(os.path.join(d, "REPORT.md"), "w", encoding="utf-8").write("\n".join(hdr + [""] + out) + "\n")
print("\n".join(hdr + [""] + out))
