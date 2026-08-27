"""
TEAM3 motor — static thrust measurement (TMS)
Reads the load-cell log and plots the thrust curve with the derived figures
that the flight software depends on.
"""
import pandas as pd, numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SRC = "tms_team3.csv"

df = pd.read_csv(SRC)
d = df[["t(ms)", "F(N)", "t0(ms)"]].dropna(subset=["t0(ms)"]).copy()
d["t0"] = d["t0(ms)"].astype(float)
b = d[d["t0"] >= 0].copy()

t = b["t0"].values / 1000.0
F = b["F(N)"].values

impulse = np.trapezoid(F, t)
burn_s = t[-1]
f_avg = impulse / burn_s
f_peak = F.max()
t_peak = t[F.argmax()]

fig, ax = plt.subplots(figsize=(9, 5))

ax.fill_between(t * 1000, F, color="#1f3864", alpha=.12, zorder=2)
ax.plot(t * 1000, F, color="#1f3864", lw=1.8, zorder=3)

ax.plot(t_peak * 1000, f_peak, "o", color="#c62828", ms=7, zorder=4)
ax.annotate(f"peak {f_peak:.1f} N", (t_peak * 1000, f_peak),
            textcoords="offset points", xytext=(14, 4),
            fontsize=9, color="#c62828")

ax.axhline(f_avg, color="#2e7d32", ls="--", lw=1.2, zorder=3)
ax.text(60, f_avg + 4, f"average {f_avg:.1f} N", fontsize=9, color="#2e7d32")

ax.axvline(burn_s * 1000, color="#6a1b9a", ls="-.", lw=1.4, zorder=3)
ax.text(burn_s * 1000 - 30, f_peak * .78,
        f"T+{burn_s*1000:.0f} ms\ndeployment inhibit\nlifts here",
        ha="right", fontsize=8.5, color="#6a1b9a")

ax.set_xlabel("Time from ignition (ms)")
ax.set_ylabel("Thrust (N)")
ax.set_title(f"TEAM3 motor static fire — {impulse:.1f} N\u00b7s total impulse (H class)",
             fontsize=12, weight="bold")
ax.set_xlim(0, burn_s * 1000 * 1.02)
ax.set_ylim(0, f_peak * 1.12)
ax.grid(alpha=.25, zorder=1)

plt.tight_layout()
plt.savefig("images/thrust-curve.png", dpi=160)
print("impulse %.2f N-s | avg %.2f N | peak %.2f N at %.0f ms | burn %.3f s"
      % (impulse, f_avg, f_peak, t_peak * 1000, burn_s))