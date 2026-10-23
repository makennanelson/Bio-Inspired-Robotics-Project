from dataclasses import replace
from pathlib import Path

from gait_sim import GAITS, SpringParameters, result_to_csv, run_simulation

OUTPUT = Path(__file__).resolve().parent / "results"
SPRINGS = SpringParameters(dimensionless_stiffness=15.0, damping_ratio=0.20)

# Compare with the current controller settings (250, 100).
GAIN_TRIALS = [
    (75.0, 25.0), (75.0, 50.0), (75.0, 75.0),
    (100.0, 25.0), (100.0, 50.0), (100.0, 75.0),
    (125.0, 25.0), (125.0, 50.0), (125.0, 75.0),
    (150.0, 25.0), (150.0, 50.0), (150.0, 75.0),
]


def main():
    results = []

    for kp, ki in GAIN_TRIALS:
        run_gait = replace(GAITS["run"], drive_kp=kp, drive_ki=ki)
        metrics = run_simulation(run_gait, SPRINGS)

        mean_speed = metrics["distance_m"] / metrics["duration_s"]
        result = {
            **metrics,
            "drive_kp": kp,
            "drive_ki": ki,
            "mean_speed_m_s": mean_speed,
        }
        results.append(result)

        print(
            f"Kp={kp:.0f}, Ki={ki:.0f}: "
            f"mean speed={mean_speed:.2f} m/s, "
            f"speed error={metrics['rms_speed_error_m_s']:.2f} m/s, "
            f"flight={metrics['flight_fraction']:.2f}, "
            f"CoT={metrics['cost_of_transport']:.2f}"
        )

    result_to_csv(results, OUTPUT / "run_controller_trials_refined.csv")
    print("Saved results to results/run_controller_trials_refined.csv")


if __name__ == "__main__":
    main()