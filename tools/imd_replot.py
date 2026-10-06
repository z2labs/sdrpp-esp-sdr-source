"""Replot the HackRF two-tone sweep: VSG60 own IM3 at the measured -53 dBc, IIP3 lower bound per gain."""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
D = sys.argv[1] if len(sys.argv) > 1 else r"D:\Measurements\ESP32-S3-SDR_speedup\sdrpp_module\imd_hackrf"
SRC = 53.0
rows = [json.loads(l) for l in open(os.path.join(D, "results.jsonl"), encoding="utf-8")]
I = [r for r in rows if r["test"] == "I"]
gains = list(dict.fromkeys(r["gain"] for r in I))
fig, axs = plt.subplots(1, len(gains), figsize=(4.6 * len(gains), 4.6), squeeze=False)
for ax, g in zip(axs[0], gains):
    r = [x for x in I if x["gain"] == g]
    pt = np.array([x["p_tone"] for x in r]); pi = np.array([x["p_im3"] for x in r])
    fl = r[0]["floor_dbfs"] - r[0]["cal"]
    ok = (pi - fl) >= 6
    # lower bound: at the strongest usable point the DUT's IM3 is at most the measured value
    lb = float(np.max(pt[ok] + (pt[ok] - pi[ok]) / 2)) if ok.any() else float("nan")
    ax.plot(pt, pt, "o-", ms=3, label="tone")
    ax.plot(pt, pi, "s-", ms=3, label="IM3 measured (input-referred)")
    ax.plot(pt, pt - SRC, "r:", lw=1.2, label="VSG60 own IM3 (-%.0f dBc)" % SRC)
    ax.axhline(fl, color="gray", ls=":", label="noise, 300 Hz")
    ax.set_title(("HackRF LNA %s / VGA %s" % tuple(v.strip() for v in g.strip("()").split(",")[:2]) if "," in g else "ESP32-S3 gain %s" % g) + ": IIP3 > %.0f dBm" % lb, fontsize=10)
    ax.set_xlabel("tone level at input [dBm]"); ax.set_ylabel("level at input [dBm]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)
fig.suptitle("Two-tone 20 kHz, 2350 MHz: the IM3 follows the generator's own -53 dBc up to the ADC limit -> IIP3 is a lower bound", fontsize=11)
fig.tight_layout(); p = os.path.join(D, "I_imd3_v2.png"); fig.savefig(p, dpi=120); os.startfile(p); print(p)
os.startfile(os.path.join(D, "K_blocking.png"))
