from pathlib import Path
import csv

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results" / "modified_slip"


def rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main():
    summary = rows(OUT / "baseline_summary.csv")
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    labels = [r["gait"].title() for r in summary]
    axes[0].bar(labels, [float(r["cost_of_transport"]) for r in summary], color=["#8c1d40", "#ffc627"])
    axes[0].set_title("Cost of transport")
    axes[0].set_ylabel("positive actuator work / (mg × distance)")
    axes[1].bar(labels, [float(r["spring_return_ratio"]) for r in summary], color=["#8c1d40", "#ffc627"])
    axes[1].set_title("Elastic return ratio")
    axes[1].set_ylim(0, 1.05)
    x = range(len(summary))
    width = 0.24
    for offset, key, label, color in [
        (-width, "single_support_fraction", "single support", "#8c1d40"),
        (0, "double_support_fraction", "double support", "#ffc627"),
        (width, "flight_fraction", "flight", "#6f7378"),
    ]:
        axes[2].bar([i + offset for i in x], [float(r[key]) for r in summary], width,
                    label=label, color=color)
    axes[2].set_xticks(list(x), labels)
    axes[2].set_ylim(0, 1)
    axes[2].set_title("Measured contact fractions")
    axes[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "baseline_comparison.png", dpi=180)

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for gait, color in (("walk", "#8c1d40"), ("run", "#d6a800")):
        data = rows(OUT / f"{gait}_baseline.csv")
        t = [float(r["time_s"]) for r in data]
        axes[0].plot(t, [float(r["com_height_m"]) for r in data], label=gait, color=color)
        axes[1].plot(t, [float(r["speed_m_s"]) for r in data], label=gait, color=color)
    axes[0].set_ylabel("COM height (m)")
    axes[1].set_ylabel("Forward speed (m/s)")
    axes[1].set_xlabel("Time (s)")
    for ax in axes:
        ax.grid(True, alpha=.25)
        ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "baseline_timeseries.png", dpi=180)


if __name__ == "__main__":
    main()
