"""Overlay hwpn.py / pn_ext.py runs:  python tools/pn_compare.py out.png DIR[=label] DIR[=label] ... [--scale-to 2350e6]
L(f) per run (62.5k below 3 kHz, 250k above), optionally scaled by 20*log10(f_target / f_measured) so a
lower-frequency run (RTL-SDR at 1.7 GHz) can sit next to 2.35 GHz ones; plus a table of L at fixed offsets."""
import argparse, glob, json, os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("png"); ap.add_argument("runs", nargs="+")
    ap.add_argument("--scale-to", type=float, default=None)
    ap.add_argument("--freq", type=float, default=None, help="only this carrier from each run (default: the first)")
    a = ap.parse_args()
    fig, ax = plt.subplots(figsize=(9, 5.5)); rows = []
    for spec in a.runs:
        d, _, lab = spec.partition("="); lab = lab or os.path.basename(os.path.normpath(d))
        recs = [json.loads(l) for l in open(os.path.join(d, "results.jsonl"), encoding="utf-8")]
        pr = [r for r in recs if r.get("test") == "P"]
        if not pr: continue
        f0 = a.freq or pr[0]["f0"]
        k = 20 * np.log10(a.scale_to / f0) if a.scale_to else 0.0
        color = None
        for rate, lo, hi in ((62500, 10, 3e3), (250000, 3e3, 1.2e5)):
            fn = os.path.join(d, "pn_%d_%d.npz" % (int(f0 / 1e6), rate))
            if not os.path.exists(fn): continue
            z = np.load(fn); m = (z["f"] >= lo) & (z["f"] < hi)
            ln, = ax.semilogx(z["f"][m], z["L"][m] + k, lw=0.8, color=color,
                              label=None if color else "%s, %.0f MHz%s" % (lab, f0 / 1e6, " (scaled %+.1f dB)" % k if k else ""))
            color = ln.get_color()
        pts = {}
        for r in pr:
            if r["f0"] == f0:
                for o, v in r["L"].items():
                    if v is not None and (int(o) < 3000) == (r["rate"] == 62500): pts[o] = v + k
        fm = [r for r in pr if r["f0"] == f0 and r["rate"] == 62500]
        rows.append((lab, f0, pts, fm[0]["res_fm_disc_hz"] if fm else None))
    ax.set_xlim(10, 1.2e5); ax.set_ylim(-130, -30); ax.grid(True, which="both", alpha=0.3)
    ax.set_xlabel("offset from carrier [Hz]"); ax.set_ylabel("L(f) [dBc/Hz]"); ax.legend(fontsize=8)
    ax.set_title("Receive-chain phase noise, VSG60 CW (source PN included)")
    fig.tight_layout(); fig.savefig(a.png, dpi=130)
    offs = ["100", "1000", "10000", "100000"]
    print("| receiver | carrier | " + " | ".join("L(%s Hz)" % o for o in offs) + " | residual FM 0.3-3 kHz |")
    print("|---|---|" + "---|" * (len(offs) + 1))
    for lab, f0, pts, fm in rows:
        print("| %s | %.0f MHz | %s | %s |" % (lab, f0 / 1e6, " | ".join(
            "%.0f" % pts[o] if o in pts else "-" for o in offs), "%.1f Hz" % fm if fm else "-"))


if __name__ == "__main__":
    main()
