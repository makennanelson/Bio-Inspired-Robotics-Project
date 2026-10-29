from pathlib import Path

from gait_sim import GAITS, SpringParameters, result_to_csv, run_simulation

OUTPUT = Path(__file__).resolve().parent / "results" / "modified_slip"

# Values to test
STIFFNESS_VALUES = [5.0, 7.5, 10.0, 12.5, 15.0]
DAMPING_VALUES = [0.05, 0.10, 0.20]

def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    trials = []

    for gait_name, gait in GAITS.items():
        for stiffness in STIFFNESS_VALUES:
            for damping in DAMPING_VALUES:
                parameters = SpringParameters(
                    dimensionless_stiffness=stiffness,
                    damping_ratio=damping,
                )

                result = run_simulation(gait, parameters)
                trials.append(result)

                print(
                    f"{gait_name}: "
                    f"kappa={stiffness:.1f}, "
                    f"zeta={damping:.2f}, "
                    f"CoT={result['cost_of_transport']:.3f}, "
                    f"stable={result['stable']}"
                )

    result_to_csv(trials, OUTPUT / "parameter_sweep.csv")
    print("Saved results to results/modified_slip/parameter_sweep.csv")

if __name__ == "__main__":
    main()
