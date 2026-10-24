"""Draw the first-test guide's plot from exported RLOG values. Requires matplotlib."""

from pathlib import Path
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


root = Path(__file__).resolve().parents[1]
assets = root / "SimLoop" / "docs" / "assets"
with (assets / "first-test-slide.csv").open(newline="", encoding="utf-8") as source:
    rows = list(csv.DictReader(source))
time = [float(row["time_s"]) for row in rows]
position = [float(row["position_in"]) for row in rows]
target = float(rows[0]["target_in"])

plt.rcParams.update({"font.size": 11, "svg.fonttype": "none", "svg.hashsalt": "simloop-first-test"})
fig, ax = plt.subplots(figsize=(9.6, 4.3), layout="constrained")
fig.set_facecolor("#17061d")
ax.set_facecolor("#17061d")
ax.axhline(target, color="#ffcb5c", linestyle=(0, (5, 4)), linewidth=1.4, label="Target: 12 in")
ax.plot(time, position, color="#f86a43", linewidth=2.4, label="Recorded simulated position")
ax.set(xlim=(0, 3), ylim=(0, 14), xlabel="Simulated time (s)", ylabel="Slide position (in)")
ax.set_title("First simulated test: slide position", loc="left", color="#f4edf5", pad=16)
ax.tick_params(colors="#cbbdce")
ax.xaxis.label.set_color("#cbbdce")
ax.yaxis.label.set_color("#cbbdce")
ax.spines[["top", "right"]].set_visible(False)
for side in ("bottom", "left"):
    ax.spines[side].set_color("#746079")
ax.grid(color="#6d5176", alpha=0.28, linewidth=0.7)
ax.legend(loc="lower right", frameon=False, labelcolor="#e8dfea", fontsize=10)
ax.annotate(
    f"Final: {position[-1]:.3f} in",
    xy=(time[-1], position[-1]),
    xytext=(2.05, 9.0),
    color="#e8dfea",
    arrowprops={"arrowstyle": "-", "color": "#ac9ab1"},
)
fig.savefig(assets / "first-test-slide.svg", metadata={"Date": None, "Creator": "SimLoop documentation"})
svg = assets / "first-test-slide.svg"
svg.write_text("\n".join(line.rstrip() for line in svg.read_text(encoding="utf-8").splitlines()) + "\n",
               encoding="utf-8")
print(f"Plotted {len(rows)} recorded frames; peak {max(position):.6f} in; final {position[-1]:.6f} in.")
