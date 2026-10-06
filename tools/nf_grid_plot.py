"""HackRF NF / clip level / dynamic range over the LNA x VGA grid, from hackrf_N/results.jsonl.
Clip level from the CW calibration (tone power 0 dBFS = full-scale complex amplitude): clip = -cal. Not the raw peak,
which at low gain is set by noise and quantisation; points where the raw ADC peak hit full scale during the
calibration (strong out-of-band signal) are marked."""
import json, os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
D = r"D:\Measurements\ESP32-S3-SDR_speedup\sdrpp_module\hackrf_N"
rows = [json.loads(l) for l in open(os.path.join(D, "results.jsonl"), encoding="utf-8") if '"N"' in l]
freqs = sorted(set(r["f0"] for r in rows))
L = [0, 8, 16, 24, 32, 40]; V = [0, 10, 20, 30, 40]
fig, axs = plt.subplots(2, len(freqs), figsize=(5.2 * len(freqs), 8.4), squeeze=False)
best = []
for j, f0 in enumerate(freqs):
    nf = np.full((len(L), len(V)), np.nan); dr = np.full_like(nf, np.nan); bad = np.zeros_like(nf, bool)
    for r in rows:
        if r["f0"] != f0: continue
        l, v = [int(s) for s in r["gain"].strip("()").split(",")]
        i, k = L.index(l), V.index(v)
        clip = -r["cal"]
        nf[i, k] = r["nf"]; dr[i, k] = clip - (r["noise_dbm_hz"] + 10 * np.log10(250e3))
        bad[i, k] = r["clip_dbm"] <= (-60 - 11.2) + 0.5          # raw peak at full scale during the -71 dBm calibration
    for ax, M, t, cm in ((axs[0][j], nf, "noise figure [dB]", "viridis_r"), (axs[1][j], dr, "clip level over noise in 250 kHz [dB]", "viridis")):
        im = ax.imshow(M, origin="lower", cmap=cm, aspect="auto")
        for i in range(len(L)):
            for k in range(len(V)):
                if np.isfinite(M[i, k]):
                    ax.text(k, i, "%.0f%s" % (M[i, k], "*" if bad[i, k] else ""), ha="center", va="center", fontsize=8,
                            color="w" if (M[i, k] > np.nanmedian(M)) == (cm == "viridis_r") else "k")
        ax.set_xticks(range(len(V))); ax.set_xticklabels(V); ax.set_yticks(range(len(L))); ax.set_yticklabels(L)
        ax.set_xlabel("VGA [dB]"); ax.set_ylabel("LNA [dB]"); ax.set_title("%.0f MHz: %s" % (f0 / 1e6, t), fontsize=10)
        fig.colorbar(im, ax=ax, shrink=0.8)
    ok = ~bad
    i, k = np.unravel_index(np.nanargmin(np.where(ok, nf, np.nan)), nf.shape)
    best.append((f0, L[i], V[k], nf[i, k], dr[i, k]))
fig.suptitle("HackRF One (RF amp off), conducted, 2 MS/s -> 250 kS/s.  * = ADC at full scale from a signal outside the measurement (2.4 GHz traffic)", fontsize=10)
fig.tight_layout(); p = os.path.join(D, "N_nf_grid.png"); fig.savefig(p, dpi=120); os.startfile(p)
for b in best: print("%.0f MHz: lowest NF %.1f dB at LNA %d / VGA %d, clip-to-noise %.1f dB in 250 kHz" % (b[0] / 1e6, b[3], b[1], b[2], b[4]))
