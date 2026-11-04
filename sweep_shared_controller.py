"""Test one blended controller over speed and ground stiffness."""
import json
from pathlib import Path

from gait_sim import result_to_csv, run_simulation
from shared_controller import make_shared_gait

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results" / "shared_controller_transition_earlier_test_2"
GROUND_KAPPAS = (10.0, 25.0, 50.0, 100.0, 200.0)
TARGET_SPEEDS = (0.85, 1.10, 1.25, 1.40, 1.55, 1.70, 1.85, 2.00)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    results = []

    for kappa in GROUND_KAPPAS:
        for speed in TARGET_SPEEDS:
            gait, params = make_shared_gait(speed, kappa)
            csv_path = OUT / f"speed_{speed:.2f}_ground_{kappa:g}.csv"
            result = run_simulation(gait, params, csv_path)
            results.append(result)
            print(json.dumps({
                "speed_m_s": speed,
                "ground_kappa": kappa,
                "duty_factor": gait.duty_factor,
                "run_blend": gait.run_blend,
                "cost_of_transport": result["cost_of_transport"],
                "speed_error_m_s": result["rms_speed_error_m_s"],
                "actual_flight": result["flight_fraction"],
                "scheduled_flight": result["scheduled_flight_fraction"],
                "completed_safety_checks": result["stable"],
            }))

    result_to_csv(results, OUT / "shared_controller_summary.csv")


if __name__ == "__main__":
    main()