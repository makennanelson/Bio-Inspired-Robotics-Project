"""Compare baseline, each gait's optimum, and the average shared parameter pair."""
from pathlib import Path
import json

from gait_sim import GAITS, SpringParameters, result_to_csv, run_simulation

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results"


def main():
    best = {name: json.loads((OUT / f"{name}_best.json").read_text())
            for name in ("walk", "run")}
    shared = SpringParameters(
        dimensionless_stiffness=(best["walk"]["dimensionless_stiffness"] +
                                 best["run"]["dimensionless_stiffness"]) / 2.0,
        damping_ratio=(best["walk"]["damping_ratio"] + best["run"]["damping_ratio"]) / 2.0,
    )
    rows = []
    for gait_name in ("walk", "run"):
        baseline = run_simulation(GAITS[gait_name], SpringParameters(15.0, 0.20))
        individual = run_simulation(
            GAITS[gait_name], SpringParameters(best[gait_name]["dimensionless_stiffness"],
                                               best[gait_name]["damping_ratio"]))
        common = run_simulation(GAITS[gait_name], shared)
        for label, result in (("baseline", baseline), ("individual_optimum", individual),
                              ("shared_parameters", common)):
            rows.append({"case": label, **result})
    result_to_csv(rows, OUT / "shared_vs_gait_specific.csv")
    print(json.dumps({"shared_dimensionless_stiffness": shared.dimensionless_stiffness,
                      "shared_damping_ratio": shared.damping_ratio,
                      "output": str(OUT / "shared_vs_gait_specific.csv")}, indent=2))


if __name__ == "__main__":
    main()
