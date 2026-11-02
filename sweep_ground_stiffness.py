"""Compare walking and running with the same dimensionless ground stiffness."""
from pathlib import Path
import json

from gait_sim import GAITS, SpringParameters, result_to_csv, run_simulation

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results" / "ground_stiffness"

# Dimensionless ground stiffness: k_ground * leg_length / (mass * gravity).
GROUND_STIFFNESS_VALUES = [10.0, 25.0, 50.0, 100.0, 200.0]
LEG_STIFFNESS = 15.0
DAMPING_RATIO = 0.20


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    results = []

    for ground_kappa in GROUND_STIFFNESS_VALUES:
        for gait_name in ("walk", "run"):
            params = SpringParameters(
                dimensionless_stiffness=LEG_STIFFNESS,
                damping_ratio=DAMPING_RATIO,
                dimensionless_ground_stiffness=ground_kappa,
            )
            time_series = OUT / f"{gait_name}_ground_kappa_{ground_kappa:g}.csv"
            result = run_simulation(GAITS[gait_name], params, time_series)
            results.append(result)
            flight_gap = abs(result["flight_fraction"] - result["scheduled_flight_fraction"])
            if flight_gap > 0.05:
                print(
                    "WARNING: actual and scheduled flight differ by "
                    f"{flight_gap:.3f} for {gait_name} at ground kappa={ground_kappa:g}. "
                    "Treat this run as a contact-schedule mismatch."
                )
            print(json.dumps({
                "gait": gait_name,
                "ground_kappa": ground_kappa,
                "leg_to_ground_stiffness_ratio": result["relative_leg_ground_stiffness"],
                "effective_kappa": result["dimensionless_effective_stiffness"],
                "cost_of_transport": result["cost_of_transport"],
                "actual_flight_fraction": result["flight_fraction"],
                "scheduled_flight_fraction": result["scheduled_flight_fraction"],
                "flight_fraction_difference": result["flight_fraction_difference"],
                "stable_flag": result["stable"],
                "termination_reason": result["termination_reason"],
            }))

    summary = OUT / "ground_stiffness_summary.csv"
    result_to_csv(results, summary)
    print(f"Saved summary to {summary}")


if __name__ == "__main__":
    main()
