"""Bounded derivative-free stiffness/damping search for each gait."""
from pathlib import Path
import csv
import json

from scipy.optimize import minimize

from gait_sim import GAITS, SpringParameters, result_to_csv, run_simulation

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results"
BOUNDS = ((5.0, 35.0), (0.05, 0.65))  # dimensionless stiffness, damping ratio


def optimize_gait(name: str):
    gait = GAITS[name]
    evaluations = []

    def objective(x):
        params = SpringParameters(float(x[0]), float(x[1]))
        metrics = run_simulation(gait, params)
        # Unstable trials receive a large penalty. A small tracking penalty
        # prevents a low-work but non-progressing solution from appearing best.
        value = metrics["cost_of_transport"]
        value += 0.15 * metrics["rms_speed_error_m_s"]
        mean_speed = metrics["distance_m"] / max(metrics["duration_s"], 1e-6)
        value += 4.0 * abs(mean_speed - gait.speed) / gait.speed
        value += 10.0 * max(0.0, metrics["max_spring_deflection_m"] - 0.15)
        if name == "walk":
            # Walking should retain support throughout the stride, with at
            # least some double-support time in the simulated contacts.
            value += 20.0 * metrics["flight_fraction"]
            value += 2.0 * max(0.0, 0.03 - metrics["double_support_fraction"])
        else:
            # Running should show an aerial interval, but not spend nearly
            # the entire cycle unsupported.
            value += 10.0 * max(0.0, 0.08 - metrics["flight_fraction"])
            value += 2.0 * max(0.0, metrics["flight_fraction"] - 0.55)
        if not metrics["stable"] or metrics["distance_m"] < 0.5 * gait.speed * gait.duration:
            value += 100.0 + max(0.0, 0.5 * gait.speed * gait.duration - metrics["distance_m"])
        evaluations.append({**metrics, "objective": value})
        print(f"{name}: kappa={x[0]:.2f}, zeta={x[1]:.3f}, J={value:.4f}, stable={metrics['stable']}")
        return value

    opt = minimize(objective, x0=[15.0, 0.20], method="Powell", bounds=BOUNDS,
                   options={"maxiter": 12, "xtol": 0.04, "ftol": 0.02, "disp": True})
    best = run_simulation(gait, SpringParameters(float(opt.x[0]), float(opt.x[1])),
                          OUT / f"{name}_optimized.csv")
    return opt, evaluations, best


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ("walk", "run"):
        opt, evaluations, best = optimize_gait(name)
        result_to_csv(evaluations, OUT / f"{name}_optimization_trials.csv")
        (OUT / f"{name}_best.json").write_text(json.dumps(best, indent=2))
        print(f"\nBest {name} candidate: {json.dumps(best, indent=2)}\n")


if __name__ == "__main__":
    main()
