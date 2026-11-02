"""Plot and diagnose a ground-stiffness sweep summary CSV."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT = ROOT / "results" / "ground_stiffness" / "ground_stiffness_summary.csv"
DEFAULT_OUTPUT = ROOT / "results" / "ground_stiffness" / "analysis"
REQUIRED = {
    "gait",
    "target_speed_m_s",
    "dimensionless_ground_stiffness",
    "cost_of_transport",
    "flight_fraction",
    "scheduled_flight_fraction",
    "stable",
}
COLORS = {"walk": "#2878B5", "run": "#D95319"}


def load_summary(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        fields = set(reader.fieldnames or [])
        missing = REQUIRED - fields
        if missing:
            raise ValueError("Summary CSV is missing columns: " + ", ".join(sorted(missing)))
        rows = list(reader)
    if not rows:
        raise ValueError(f"No data rows found in {path}")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT,
                        help="ground_stiffness_summary.csv from the sweep")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    rows = load_summary(args.input)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    gaits = sorted({row["gait"] for row in rows})
    kappas = sorted({float(row["dimensionless_ground_stiffness"]) for row in rows})

    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    for gait in gaits:
        subset = sorted((r for r in rows if r["gait"] == gait),
                        key=lambda r: float(r["dimensionless_ground_stiffness"]))
        ax.plot([float(r["dimensionless_ground_stiffness"]) for r in subset],
                [float(r["cost_of_transport"]) for r in subset],
                marker="o", linewidth=2, color=COLORS.get(gait), label=gait.title())
    ax.set(xlabel="Dimensionless ground stiffness, κ_ground",
           ylabel="Cost of transport", title="Energy cost across ground stiffness")
    ax.set_xticks(kappas)
    ax.grid(True, alpha=0.3)
    ax.legend(title="Gait")
    cot_path = args.output_dir / "cost_of_transport_vs_ground_stiffness.png"
    fig.savefig(cot_path, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    for gait in gaits:
        subset = sorted((r for r in rows if r["gait"] == gait),
                        key=lambda r: float(r["dimensionless_ground_stiffness"]))
        x = [float(r["dimensionless_ground_stiffness"]) for r in subset]
        color = COLORS.get(gait)
        ax.plot(x, [float(r["flight_fraction"]) for r in subset], marker="o",
                linewidth=2, color=color, label=f"{gait.title()} actual")
        ax.plot(x, [float(r["scheduled_flight_fraction"]) for r in subset], marker="x",
                linewidth=1.5, linestyle="--", color=color,
                label=f"{gait.title()} scheduled")
    ax.set(xlabel="Dimensionless ground stiffness, κ_ground",
           ylabel="Fraction of simulation in flight", ylim=(-0.02, 1.02),
           title="Scheduled flight versus measured foot-ground contact")
    ax.set_xticks(kappas)
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2)
    flight_path = args.output_dir / "flight_fraction_vs_ground_stiffness.png"
    fig.savefig(flight_path, dpi=180)
    plt.close(fig)

    diagnostic_path = args.output_dir / "flight_schedule_diagnostics.csv"
    with diagnostic_path.open("w", newline="", encoding="utf-8") as file:
        fields = ["gait", "target_speed_m_s", "dimensionless_ground_stiffness",
                  "actual_flight_fraction", "scheduled_flight_fraction",
                  "absolute_flight_gap", "gap_over_0.05", "stable_guard",
                  "termination_reason"]
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for row in sorted(rows, key=lambda r: (r["gait"],
                                               float(r["dimensionless_ground_stiffness"]))):
            actual = float(row["flight_fraction"])
            scheduled = float(row["scheduled_flight_fraction"])
            writer.writerow({
                "gait": row["gait"],
                "target_speed_m_s": row["target_speed_m_s"],
                "dimensionless_ground_stiffness": row["dimensionless_ground_stiffness"],
                "actual_flight_fraction": actual,
                "scheduled_flight_fraction": scheduled,
                "absolute_flight_gap": abs(actual - scheduled),
                "gap_over_0.05": abs(actual - scheduled) > 0.05,
                "stable_guard": row["stable"],
                "termination_reason": row.get("termination_reason", "not_recorded"),
            })

    print(f"Saved: {cot_path}")
    print(f"Saved: {flight_path}")
    print(f"Saved: {diagnostic_path}")
    print("A flight gap above 0.05 is flagged for review; it is not an automatic failure criterion.")


if __name__ == "__main__":
    main()
