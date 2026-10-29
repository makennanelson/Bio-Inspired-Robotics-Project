from pathlib import Path
import csv
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
data_file = ROOT / "results/modified_slip/parameter_sweep.csv"
output_dir = ROOT / "results/modified_slip/analysis"
output_dir.mkdir(parents=True, exist_ok=True)

with data_file.open(newline="") as f:
    rows = list(csv.DictReader(f))

for row in rows:
    for key in (
        "dimensionless_stiffness",
        "damping_ratio",
        "cost_of_transport",
        "rms_speed_error_m_s",
        "flight_fraction",
        "max_spring_deflection_m",
    ):
        row[key] = float(row[key])

metrics = [
    ("cost_of_transport", "Cost of transport", None),
    ("flight_fraction", "Flight fraction", "flight"),
    ("rms_speed_error_m_s", "RMS speed error (m/s)", "speed"),
    ("max_spring_deflection_m", "Maximum spring deflection (m)", "deflection"),
]

for metric, title, limit_type in metrics:
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    for ax, gait in zip(axes, ("walk", "run")):
        gait_rows = [r for r in rows if r["gait"] == gait]
        damping_values = sorted({r["damping_ratio"] for r in gait_rows})

        for damping in damping_values:
            group = sorted(
                [r for r in gait_rows if r["damping_ratio"] == damping],
                key=lambda r: r["dimensionless_stiffness"],
            )
            ax.plot(
                [r["dimensionless_stiffness"] for r in group],
                [r[metric] for r in group],
                marker="o",
                label=f"ζ = {damping:g}",
            )

        if limit_type == "flight":
            if gait == "walk":
                ax.axhline(0.05, color="red", linestyle="--", label="walk flight limit")
            else:
                ax.axhline(0.08, color="red", linestyle="--", label="minimum run flight")
                ax.axhline(0.55, color="darkred", linestyle=":", label="maximum run flight")
        elif limit_type == "speed":
            target = 0.85 if gait == "walk" else 2.0
            ax.axhline(0.10 * target, color="red", linestyle="--")
        elif limit_type == "deflection":
            ax.axhline(0.195, color="red", linestyle="--")

        ax.set_title(gait.capitalize())
        ax.set_xlabel("Dimensionless stiffness κ")
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.3)
        ax.legend()

    fig.tight_layout()
    filename = metric + ".png"
    fig.savefig(output_dir / filename, dpi=200)
    plt.close(fig)
    print(f"Saved results/modified_slip/analysis/{filename}")
