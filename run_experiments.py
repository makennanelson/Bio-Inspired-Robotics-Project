from pathlib import Path
import json

from gait_sim import GAITS, SpringParameters, result_to_csv, run_simulation

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results"


def main():
    # First pass is a transparent baseline, not an asserted optimum.
    params = SpringParameters(dimensionless_stiffness=15.0, damping_ratio=0.20)
    results = []
    for name in ("walk", "run"):
        result = run_simulation(GAITS[name], params, OUT / f"{name}_baseline.csv")
        results.append(result)
        print(json.dumps(result, indent=2))
    result_to_csv(results, OUT / "baseline_summary.csv")


if __name__ == "__main__":
    main()
