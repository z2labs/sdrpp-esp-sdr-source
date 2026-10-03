"""Extra report sections (hwlong.py G/L/N/R/S/F/W and hwextra.py T/I/K/M/Q). Called from hwreport.py."""
import numpy as np
import matplotlib.pyplot as plt


def sections(d, T, say, fig):
    t0 = T("start")[0]["t"] if T("start") else 0
    # ---- G: gain curve ----
    G = T("G")
    if G:
        say("## G. Gain curve (62.5 kS/s, 16-bit link, CW -50 dBm)\n")
        g = np.array([r["gain"] for r in G]); lv = np.array([r["level"] for r in G]); fl = np.array([r["floor_bin"] for r in G])
        Go = T("Goff")
        f, ax = plt.subplots(1, 2, figsize=(11, 4))
        ax[0].plot(g, lv, "o-", label="tone"); ax[0].set_xlabel("gain index"); ax[0].set_ylabel("tone [dBFS]"); ax[0].grid(alpha=.3)
        ax[1].plot(g, lv - fl, "o-", label="tone / bin floor"); ax[1].set_xlabel("gain index"); ax[1].set_ylabel("dB"); ax[1].grid(alpha=.3)
        if Go:
            ax2 = ax[0].twinx(); ax2.plot([r["gain"] for r in Go], [r["floor_bin"] for r in Go], "s--", c="C3", ms=4)
            ax2.set_ylabel("VSG off: bin floor [dBFS]", color="C3")
        sl = np.polyfit(g[(g >= 20) & (g <= 60)], lv[(g >= 20) & (g <= 60)], 1)[0] if np.sum((g >= 20) & (g <= 60)) > 3 else np.nan
        k = int(np.argmax(lv - fl))
        say("- gain range 0-82: tone %.1f ... %.1f dBFS (%.1f dB span), mean slope %.2f dB per index step (20-60)" % (lv.min(), lv.max(), np.ptp(lv), sl))
        say("- best tone/floor %.1f dB at gain %d" % ((lv - fl)[k], g[k]))
        say("\n![G](%s)\n" % fig("G_gain.png"))
    # ---- L: linearity / MDS ----
    L = T("L")
    if L:
        say("## L. Level linearity down to the noise (VSG -50 ... -130 dBm, near-field: relative input)\n")
        f, ax = plt.subplots(1, 2, figsize=(11, 4))
        for mode, axi in (("iq62k", 0), ("spec80", 1)):
            for g in sorted(set(r["gain"] for r in L if r["mode"] == mode)):
                rr = sorted([r for r in L if r["mode"] == mode and r["gain"] == g], key=lambda r: -r["p"])
                p = np.array([r["p"] for r in rr]); lv = np.array([r["level"] for r in rr])
                ax[axi].plot(p, lv, "o-", ms=3, label="gain %d" % g)
                plateau = np.median(np.sort(lv)[:4])   # where the tone has sunk into the floor (or a residual line)
                mid = (lv > plateau + 20) & (lv < lv.max() - 6)
                ref = np.median((lv - p)[mid]) if np.any(mid) else lv[0] - p[0]
                dev = lv - (p + ref)
                inside = np.abs(dev) < 1.0
                hi = p[inside].max() if np.any(inside) else np.nan
                lo = p[inside].min() if np.any(inside) else np.nan
                comp = dev[0]
                what = "IQ 62.5k (1 Hz bins)" if mode == "iq62k" else "SPEC 80 MHz / 1024 (78 kHz bins)"
                say("- %s, gain %d: within 1 dB of a straight line from %.0f to %.0f dBm at the VSG = %.0f dB linear range; "
                    "at -50 dBm %+.1f dB (compression); below that the reading stays at %.1f dBFS" % (what, g, hi, lo, hi - lo, comp, plateau))
            ax[axi].set_xlabel("VSG level [dBm]"); ax[axi].set_ylabel("measured [dBFS]"); ax[axi].grid(alpha=.3); ax[axi].legend()
            ax[axi].set_title("IQ 62.5 kS/s (1 s, 1 Hz bins)" if axi == 0 else "SPEC 80 MHz / 1024 bins")
        say("\n![L](%s)\n" % fig("L_linearity.png"))
    # ---- N / Niq: noise ----
    N = T("N"); Niq = T("Niq")
    if N or Niq:
        say("## N. Noise floor\n")
        if Niq:
            say("| gain | rate | noise density [dBFS/Hz] |\n| --- | --- | --- |")
            for r in Niq: say("| %d | %g kS/s | %.1f |" % (r["gain"], r["rate"] / 1e3, r["dbfs_hz"]))
            say("")
        if N:
            say("- SPEC 80 MHz / 1024 median vs gain: " + ", ".join("%d: %.1f" % (r["gain"], r["median"]) for r in N) + " dBFS")
    # ---- R: SPEC rates ----
    Rr = T("R")
    if Rr:
        say("\n## R. SPEC profiles: spectra per second to the host\n")
        say("| span | bins | detector | spectra/s | CRC | gaps |\n| --- | --- | --- | --- | --- | --- |")
        for r in Rr: say("| %g MHz | %d | %s | %.1f | %d | %d |" % (r["fs"] / 1e6, r["bins"], "max hold" if r["maxhold"] else "mean", r["rate"], r["crc"], r["gaps"]))
        say("")
    # ---- S: settling ----
    S = T("S")
    if S:
        say("## S. After a retune: tone frequency and level in 2 ms slices\n")
        f, ax = plt.subplots(1, 2, figsize=(11, 4))
        worst = 0.0
        for r in S:
            tt = np.arange(len(r["freq_err"])) * 2
            ax[0].plot(tt, r["freq_err"], lw=.7, alpha=.6); ax[1].plot(tt, np.array(r["level"]) - np.median(r["level"]), lw=.7, alpha=.6)
            fe = np.abs(np.array(r["freq_err"]) - np.median(r["freq_err"]))
            bad = np.where(fe > 50)[0]
            worst = max(worst, (bad[-1] + 1) * 2 if len(bad) else 0)
        ax[0].set_ylim(-500, 500); ax[0].set_xlabel("ms after the first sample"); ax[0].set_ylabel("tone frequency error [Hz]")
        ax[1].set_ylim(-6, 3); ax[1].set_xlabel("ms after the first sample"); ax[1].set_ylabel("level rel. median [dB]")
        for a in ax: a.grid(alpha=.3)
        say("- %d retunes (hops 0.1 ... 200 MHz): the first sample delivered is already within 50 Hz after %d ms at the latest; first data %.0f ... %.0f ms after the update" % (
            len(S), worst, min(r["first_data_s"] for r in S) * 1e3, max(r["first_data_s"] for r in S) * 1e3))
        say("\n![S](%s)\n" % fig("S_settling.png"))
    # ---- F: phase noise ----
    F = T("F")
    if F:
        say("## F. Close-in noise of a received CW (VSG60 phase noise included)\n")
        f, ax = plt.subplots(figsize=(9, 4))
        for r in F:
            fx = np.array(r["f"]); Lx = np.array(r["L"]); m = fx > 0
            ax.semilogx(fx[m], Lx[m], lw=.8, label="%g kS/s" % (r["rate"] / 1e3))
            say("- %g kS/s: " % (r["rate"] / 1e3) + ", ".join("%s Hz: %.0f dBc/Hz" % (k, v) for k, v in r["pn"].items()))
        ax.set_xlabel("offset [Hz]"); ax.set_ylabel("dBc/Hz"); ax.grid(alpha=.3, which="both"); ax.legend()
        say("\n![F](%s)\n" % fig("F_phase_noise.png"))
    # ---- W: long soak ----
    W = T("W")
    if W:
        say("## W. Long soak (20 min IQ / SPEC blocks, check every minute)\n")
        el = np.array([r["el"] for r in W]) / 3600
        iq = [r for r in W if r["kind"] == "iq" and "err_hz" in r]
        sp = [r for r in W if r["kind"] == "spec" and "level" in r]
        f, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        ax[0].plot([r["el"] / 3600 for r in iq], [r["err_hz"] for r in iq], ".", ms=4); ax[0].set_ylabel("IQ tone error [Hz]")
        ax[1].plot([r["el"] / 3600 for r in iq], [r["level"] for r in iq], ".", ms=4, label="IQ")
        ax[1].plot([r["el"] / 3600 for r in sp], [r["level"] for r in sp], ".", ms=4, label="SPEC"); ax[1].set_ylabel("level [dBFS]"); ax[1].legend()
        ax[2].plot(el, [r["crc"] + r["gaps"] + r["lost"] for r in W], ".", ms=3); ax[2].set_ylabel("CRC+gaps+lost (per block)")
        ax[2].set_xlabel("hours since start")
        for a in ax: a.grid(alpha=.3)
        blocks = {}
        for r in W: blocks[(r["cyc"], r["kind"])] = r
        fr = sum(r["frames"] for r in blocks.values()); sm = sum(r["samples"] for r in blocks.values())
        mini = T("Wmini")
        e = np.array([r["err_hz"] for r in iq])
        say("- %.1f h, %d checks, %d blocks, %d frames, %.2e IQ samples: CRC %d, gaps %d, lost %d; client never stopped" % (
            (W[-1]["el"] - W[0]["el"]) / 3600 + 1 / 60, len(W), len(blocks), fr, sm, max(r["crc"] for r in W), max(r["gaps"] for r in W), max(r["lost"] for r in W)))
        say("- frequency (PPM calibrated once at the start): %.0f ... %+.0f Hz at 2350 MHz (%.2f ... %+.2f ppm)" % (e.min(), e.max(), e.min() / 2350, e.max() / 2350))
        if mini: say("- %d mini rounds (LO sweep, 30 retunes, mode switches, 5 stop/start): %d checks, %d failed" % (len(mini), sum(r["n"] for r in mini), sum(r["fails"] for r in mini)))
        say("\n![W](%s)\n" % fig("W_soak.png"))
    # ---- T: Allan deviation ----
    for r in T("T"):
        say("## T. Allan deviation (gapless %.0f s capture, S3 crystal vs VSG60 reference)\n" % r["secs"])
        tau = np.array(r["tau"]); ad = np.array(r["adev"])
        f, ax = plt.subplots(figsize=(8, 4.5))
        ax.loglog(tau, ad, "o-", ms=3); ax.set_xlabel("tau [s]"); ax.set_ylabel("overlapping ADEV")
        ax.grid(alpha=.3, which="both")
        pts = [(t, float(np.interp(np.log(t), np.log(tau), ad))) for t in (0.1, 1, 10, 100, 1000) if tau.min() <= t <= tau.max()]
        say("- " + ", ".join("ADEV(%g s) = %.1e" % p for p in pts))
        say("- drift %.1f Hz/h at %.0f MHz (%.3f ppm/h); CRC %d, gaps %d, lost %d during the capture" % (
            r["drift_hz_per_hour"], r["f0"] / 1e6, r["drift_hz_per_hour"] / r["f0"] * 1e6, r["crc"], r["gaps"], r["lost"]))
        say("- short tau is set by the CW's SNR, long tau by temperature; the VSG60's own TCXO is in the result as well")
        say("\n![T](%s)\n" % fig("T_adev.png"))
    # ---- I: two-tone IMD3 ----
    I = T("I")
    if I:
        say("## I. Two-tone IMD3 (VSG60 multitone, 2 tones)\n")
        f, ax = plt.subplots(1, 2, figsize=(11, 4))
        for rate in sorted(set(r["rate"] for r in I), reverse=True):
            gs = sorted([r for r in I if r["rate"] == rate and r["sweep"] == "gain"], key=lambda r: r["gain"])
            ps = sorted([r for r in I if r["rate"] == rate and r["sweep"] == "power"], key=lambda r: -r["p"])
            lab = "%g kS/s" % (rate / 1e3)
            if gs:
                im = [max(r["im3_lo"], r["im3_hi"]) - (r["t1"] + r["t2"]) / 2 for r in gs]
                ax[0].plot([r["gain"] for r in gs], im, "o-", ms=3, label=lab)
                k = int(np.argmin(im))
                say("- %s, VSG -50 dBm: IM3 %.1f dBc at gain %d (best), %.1f dBc at gain %d (max)" % (lab, im[k], gs[k]["gain"], im[-1], gs[-1]["gain"]))
            if ps:
                p = np.array([r["p"] for r in ps]); t = np.array([(r["t1"] + r["t2"]) / 2 for r in ps])
                i3 = np.array([max(r["im3_lo"], r["im3_hi"]) for r in ps]); fl = np.array([r["floor9"] for r in ps])
                ax[1].plot(p, t, "o-", ms=3, label=lab + " tone"); ax[1].plot(p, i3, "s--", ms=3, label=lab + " IM3")
                ok = i3 > fl + 6
                if np.sum(ok) >= 3:
                    s3 = np.polyfit(p[ok], i3[ok], 1)[0]; s1 = np.polyfit(p[ok], t[ok], 1)[0]
                    oip = (t[ok] + (t[ok] - i3[ok]) / 2).mean()
                    say("- %s, gain %d: tone slope %.2f, IM3 slope %.2f (3 = ideal); extrapolated intercept %.1f dBFS (output-referred)" % (lab, ps[0]["gain"], s1, s3, oip))
                else:
                    say("- %s, gain %d: IM3 below the floor over most of the level sweep" % (lab, ps[0]["gain"]))
        ax[0].set_xlabel("gain index"); ax[0].set_ylabel("IM3 [dBc]"); ax[0].grid(alpha=.3); ax[0].legend()
        ax[1].set_xlabel("VSG level per pair [dBm]"); ax[1].set_ylabel("dBFS"); ax[1].grid(alpha=.3); ax[1].legend(fontsize=7)
        say("\n![I](%s)\n" % fig("I_imd3.png"))
    # ---- K: blocking ----
    K = [r for r in T("K") if r.get("vsg")]
    if K:
        say("## K. Out-of-band CW blocker (-50 dBm at the VSG): in-band noise-floor rise and leakage\n")
        f, ax = plt.subplots(figsize=(9, 4))
        for rate in sorted(set(r["rate"] for r in K), reverse=True):
            rr = sorted([r for r in K if r["rate"] == rate], key=lambda r: r["off"])
            ax.plot([r["off"] / 1e6 for r in rr], [r["rise"] for r in rr], "o-", ms=4, label="%g kS/s" % (rate / 1e3))
            worst = max(rr, key=lambda r: r["rise"])
            say("- %g kS/s: floor rise %.1f dB worst (blocker at %+.1f MHz); " % (rate / 1e3, worst["rise"], worst["off"] / 1e6) +
                ", ".join("%+.1f MHz: %+.1f dB" % (r["off"] / 1e6, r["rise"]) for r in rr if abs(abs(r["off"]) - 8e6) < 1 or abs(abs(r["off"]) - 4e6) < 1))
            img = [r for r in rr if abs(r["off"] + 8e6) < 1]
            if img:
                say("  - blocker at -8 MHz (image of the fs/4 IF): strongest in-band line %.1f dBFS at %+.1f kHz" % (img[0]["strongest"], img[0]["strongest_freq"] / 1e3))
        ax.set_xscale("symlog", linthresh=0.5); ax.set_xlabel("blocker offset [MHz]"); ax.set_ylabel("in-band floor rise [dB]")
        ax.grid(alpha=.3); ax.legend()
        say("\n![K](%s)\n" % fig("K_blocking.png"))
    # ---- M: latency ----
    M = [r for r in T("M") if r.get("found")]
    if M:
        say("## M. Latency: VSG output on -> first affected data at the host\n")
        say("| mode | n | median [ms] | min [ms] | max [ms] |\n| --- | --- | --- | --- | --- |")
        for mode in sorted(set(tuple(r["mode"]) for r in M), key=str):
            v = np.array([r["latency_ms"] for r in M if tuple(r["mode"]) == mode])
            lab = "SPEC %g MHz / %d" % (mode[0] / 1e6, mode[2]) if mode[0] else "IQ %g kS/s" % (mode[1] / 1e3)
            say("| %s | %d | %.1f | %.1f | %.1f |" % (lab, len(v), np.median(v), v.min(), v.max()))
        say("\nIncludes the VSG60's own switching time (upper bound for the receive path); clock offset emulator vs script %.2f ms.\n" % np.median([r["clk_offset_ms"] for r in M]))
    # ---- Q: IQ imbalance vs LO ----
    Q = T("Q"); Qi = T("Qiq")
    if Q:
        say("## Q. IQ imbalance across the tuning range\n")
        f, ax = plt.subplots(figsize=(9, 4))
        for off in sorted(set(r["off"] for r in Q)):
            rr = sorted([r for r in Q if r["off"] == off and r["mirror_db"] is not None], key=lambda r: r["f"])
            ax.plot([r["f"] / 1e6 for r in rr], [r["mirror_db"] for r in rr], "o-", ms=3, label="SPEC mirror, +%g MHz" % (off / 1e6))
        if Qi:
            ax.plot([r["f"] / 1e6 for r in Qi], [r["image_dbc"] for r in Qi], "k^-", ms=4, label="IQ 250k image (+40 kHz)")
        ax.set_xlabel("centre frequency [MHz]"); ax.set_ylabel("mirror / image [dBc]"); ax.grid(alpha=.3); ax.legend(fontsize=7)
        m = [r["mirror_db"] for r in Q if r["mirror_db"] is not None]
        say("- SPEC (LO at the centre, no fs/4): mirror %.1f ... %.1f dBc over 2230-2780 MHz and +5 ... +30 MHz offsets" % (min(m), max(m)))
        if Qi: say("- IQ mode (fs/4 IF): in-band image %.1f ... %.1f dBc" % (min(r["image_dbc"] for r in Qi), max(r["image_dbc"] for r in Qi)))
        say("\n![Q](%s)\n" % fig("Q_iq_imbalance.png"))
