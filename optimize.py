"""Grid search over dimensionless leg stiffness and touchdown angle.

Run with: python optimize.py

The first pass is a transparent grid search rather than a black-box optimizer:
every tested pair is recorded, and the objective weights are easy to inspect.
"""
from dataclasses import replace
from pathlib import Path
import csv
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gait_sim import GAITS, SpringParameters, run_simulation

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "results" / "stiffness_touchdown_optimization"
KAPPA_VALUES = (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0)
TOUCHDOWN_ANGLES_DEG = (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0)
FIXED_DAMPING_RATIO = 0.20
PEAK_FORCE_WEIGHT = 0.75
TIMING_WEIGHT = 0.25


def score(metrics, baseline, force_weight=PEAK_FORCE_WEIGHT,
          timing_weight=TIMING_WEIGHT):
    """Weighted, dimensionless score; lower is better.

    Force is peak total foot ground-reaction force divided by body weight.
    Timing is time-to-that-peak divided by sqrt(leg_length / gravity).
    The timing term penalizes relative change from baseline, so it does not
    reward an arbitrarily early impact peak.
    """
    force_ratio = metrics["peak_ground_force_body_weights"] / max(
        baseline["peak_ground_force_body_weights"], 1e-9)
    tau = metrics["dimensionless_time_to_peak_ground_force"]
    baseline_tau = baseline["dimensionless_time_to_peak_ground_force"]
    timing_change = abs(tau - baseline_tau) / max(abs(baseline_tau), 1e-9)
    value = force_weight * force_ratio + timing_weight * timing_change
    minimum_progress = 0.5 * metrics["target_speed_m_s"] * metrics["duration_s"]
    if not metrics["stable"] or metrics["distance_m"] < minimum_progress:
        value += 100.0
    return value, force_ratio, timing_change


def optimize_gait(name):
    gait = GAITS[name]
    baseline_gait = replace(gait, touchdown_angle_deg=17.1887338539)
    baseline = run_simulation(
        baseline_gait, SpringParameters(15.0, FIXED_DAMPING_RATIO)
    )
    if baseline["peak_ground_force_body_weights"] <= 0:
        raise RuntimeError(f"{name} baseline produced no measurable foot force")

    trials = []

    def evaluate_grid(kappas, angles, stage):
        for kappa in kappas:
            for angle in angles:
                candidate_gait = replace(gait, touchdown_angle_deg=angle)
                metrics = run_simulation(
                    candidate_gait,
                    SpringParameters(kappa, FIXED_DAMPING_RATIO),
                )
                objective, force_ratio, timing_change = score(metrics, baseline)
                progress_ok = bool(metrics["distance_m"] >= (
                    0.5 * metrics["target_speed_m_s"] * metrics["duration_s"]))
                row = {
                    **metrics,
                    "dimensionless_stiffness": kappa,
                    "touchdown_angle_deg": angle,
                    "objective": objective,
                    "progress_ok": progress_ok,
                    "peak_force_ratio_to_baseline": force_ratio,
                    "relative_dimensionless_timing_change": timing_change,
                    "peak_force_weight": PEAK_FORCE_WEIGHT,
                    "timing_weight": TIMING_WEIGHT,
                    "search_stage": stage,
                    "baseline_peak_force_body_weights": baseline[
                        "peak_ground_force_body_weights"],
                    "baseline_dimensionless_time_to_peak_force": baseline[
                        "dimensionless_time_to_peak_ground_force"],
                }
                trials.append(row)
                print(
                    f"{name}: kappa={kappa:4.1f}, touchdown={angle:4.1f} deg, "
                    f"peak={metrics['peak_ground_force_body_weights']:.3f} BW, "
                    f"tau_force_peak={metrics['dimensionless_time_to_peak_ground_force']:.3f}, "
                f"actual touchdown={metrics['mean_actual_touchdown_angle_deg']:.1f} deg, "
                f"J={objective:.4f}, stable={metrics['stable']}"
                )

    evaluate_grid(KAPPA_VALUES, TOUCHDOWN_ANGLES_DEG, "coarse")
    coarse_best = min(trials, key=lambda row: row["objective"])
    fine_kappas = sorted({round(min(40.0, max(2.0, coarse_best["dimensionless_stiffness"] + offset)), 3)
                          for offset in (-5.0, -2.5, 0.0, 2.5, 5.0)})
    fine_angles = sorted({round(min(40.0, max(0.0, coarse_best["touchdown_angle_deg"] + offset)), 3)
                          for offset in (-5.0, -2.5, 0.0, 2.5, 5.0)})
    evaluate_grid(fine_kappas, fine_angles, "refined")
    refined_best = min(trials, key=lambda row: row["objective"])
    finer_kappas = sorted({round(min(40.0, max(0.5, refined_best["dimensionless_stiffness"] + offset)), 3)
                           for offset in (-2.5, -1.25, 0.0, 1.25, 2.5)})
    finer_angles = sorted({round(min(45.0, max(0.0, refined_best["touchdown_angle_deg"] + offset)), 3)
                           for offset in (-2.5, -1.25, 0.0, 1.25, 2.5)})
    evaluate_grid(finer_kappas, finer_angles, "fine_refinement")

    best = min(trials, key=lambda row: row["objective"])
    return baseline, trials, best


def write_heatmap(name, trials):
    angles = sorted({row["touchdown_angle_deg"] for row in trials})
    kappas = sorted({row["dimensionless_stiffness"] for row in trials})
    values = np.full((len(angles), len(kappas)), np.nan)
    for row in trials:
        i = angles.index(row["touchdown_angle_deg"])
        j = kappas.index(row["dimensionless_stiffness"])
        if np.isnan(values[i, j]) or row["objective"] < values[i, j]:
            values[i, j] = row["objective"]
    fig, ax = plt.subplots(figsize=(8, 5.5), constrained_layout=True)
    image = ax.imshow(values, origin="lower", aspect="auto", cmap="viridis")
    ax.set_xticks(range(len(kappas)), kappas)
    ax.set_yticks(range(len(angles)), angles)
    ax.set_xlabel("Dimensionless leg stiffness, κ = kL/(mg)")
    ax.set_ylabel("Forward touchdown angle (degrees)")
    ax.set_title(f"{name.title()} gait: weighted objective (lower is better)")
    fig.colorbar(image, ax=ax, label="Objective J")
    best = min(trials, key=lambda row: row["objective"])
    ax.scatter(kappas.index(best["dimensionless_stiffness"]),
               angles.index(best["touchdown_angle_deg"]),
               marker="*", s=180, color="red", edgecolor="white", linewidth=0.8)
    fig.savefig(OUT / f"{name}_objective_heatmap.png", dpi=180)
    plt.close(fig)


def assess_tradeoffs(trials):
    """Return weight-sensitivity winners and the feasible Pareto frontier."""
    feasible = [row for row in trials if row["stable"] and row["progress_ok"]]
    sensitivity = []
    for force_weight, timing_weight in ((1.0, 0.0), (0.75, 0.25),
                                        (0.5, 0.5), (0.25, 0.75),
                                        (0.0, 1.0)):
        def weighted_value(row):
            return (force_weight * row["peak_force_ratio_to_baseline"] +
                    timing_weight * row["relative_dimensionless_timing_change"])
        winner = min(feasible, key=weighted_value)
        sensitivity.append({
            "peak_force_weight": force_weight,
            "dimensionless_timing_weight": timing_weight,
            "objective": weighted_value(winner),
            "dimensionless_stiffness": winner["dimensionless_stiffness"],
            "touchdown_angle_target_deg": winner["touchdown_angle_deg"],
            "actual_mean_touchdown_angle_deg": winner["mean_actual_touchdown_angle_deg"],
            "peak_ground_force_body_weights": winner["peak_ground_force_body_weights"],
            "dimensionless_time_to_peak_ground_force": winner[
                "dimensionless_time_to_peak_ground_force"],
            "search_stage": winner["search_stage"],
        })

    frontier = []
    for candidate in feasible:
        force = candidate["peak_force_ratio_to_baseline"]
        timing = candidate["relative_dimensionless_timing_change"]
        dominated = any(
            other["peak_force_ratio_to_baseline"] <= force and
            other["relative_dimensionless_timing_change"] <= timing and
            (other["peak_force_ratio_to_baseline"] < force or
             other["relative_dimensionless_timing_change"] < timing)
            for other in feasible
        )
        if not dominated:
            frontier.append(candidate)
    return sensitivity, frontier


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    report = [
        "# Spring stiffness and touchdown angle search",
        "",
        "The study tests dimensionless leg stiffness κ = kL/(mg) and the forward touchdown angle.",
        "Touchdown angle is the swing controller's target; the simulation separately measures the actual world-frame leg angle at foot-contact transitions.",
        "Damping ratio is held at 0.20, and the rest of each gait controller is fixed.",
        "Peak force is the maximum summed normal force from the two feet, divided by body weight.",
        "Dimensionless event time is τ = t_peak / sqrt(L/g) = t_peak * sqrt(g/L), with L = 0.74 m.",
        "The optimized timing event is when the maximum total foot force occurs; spring-compression peak timing is reported separately.",
        "The timing term penalizes relative deviation from baseline peak-force timing rather than rewarding an earlier impact.",
        "Candidates must also complete the simulation and travel at least half the target distance; otherwise they receive a large penalty.",
        f"Objective: J = {PEAK_FORCE_WEIGHT:.2f} × (candidate peak force / baseline peak force) + ",
        f"{TIMING_WEIGHT:.2f} × relative change in dimensionless peak-force time; unstable trials add 100.",
        "",
        "Each gait is searched independently on a 7 × 7 coarse grid, then two local 5 × 5 refinements around the best candidates.",
        "",
    ]
    best_results = {}
    sensitivity_rows = []
    pareto_rows = []
    for name in ("walk", "run"):
        baseline, trials, best = optimize_gait(name)
        sensitivity, frontier = assess_tradeoffs(trials)
        best_results[name] = {
            "baseline": baseline,
            "best_75_25": best,
            "weight_sensitivity": sensitivity,
            "pareto_frontier_candidate_count": len(frontier),
        }
        sensitivity_rows.extend({"gait": name, **row} for row in sensitivity)
        pareto_rows.extend({
            "gait": name,
            "dimensionless_stiffness": row["dimensionless_stiffness"],
            "touchdown_angle_target_deg": row["touchdown_angle_deg"],
            "actual_mean_touchdown_angle_deg": row["mean_actual_touchdown_angle_deg"],
            "peak_force_ratio_to_baseline": row["peak_force_ratio_to_baseline"],
            "relative_dimensionless_timing_change": row[
                "relative_dimensionless_timing_change"],
            "peak_ground_force_body_weights": row[
                "peak_ground_force_body_weights"],
            "dimensionless_time_to_peak_ground_force": row[
                "dimensionless_time_to_peak_ground_force"],
            "search_stage": row["search_stage"],
        } for row in frontier)
        csv_path = OUT / f"{name}_trials.csv"
        with csv_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(trials[0]))
            writer.writeheader()
            writer.writerows(trials)
        write_heatmap(name, trials)
        report.extend([
            f"### Weight sensitivity and trade-offs ({name})",
            "",
            "| Force / timing weights | Best κ | Target touchdown | Measured touchdown | Peak force (BW) | Dimensionless peak time |",
            "|---|---:|---:|---:|---:|---:|",
        ])
        for result in sensitivity:
            report.append(
                f"| {result['peak_force_weight']:.0%} / {result['dimensionless_timing_weight']:.0%} "
                f"| {result['dimensionless_stiffness']:.2f} "
                f"| {result['touchdown_angle_target_deg']:.2f}° "
                f"| {result['actual_mean_touchdown_angle_deg']:.1f}° "
                f"| {result['peak_ground_force_body_weights']:.2f} "
                f"| {result['dimensionless_time_to_peak_ground_force']:.3f} |"
            )
        report.append(
            f"Pareto frontier size: {len(frontier)} feasible settings; see `pareto_frontier.csv` for non-dominated force/timing trade-offs."
        )
        report.append("")
        report.extend([
            f"## {name.title()}",
            "",
            f"Baseline: peak force {baseline['peak_ground_force_body_weights']:.3f} body weights; ",
            f"dimensionless time to peak force {baseline['dimensionless_time_to_peak_ground_force']:.3f}.",
            f"Best grid point: κ = {best['dimensionless_stiffness']:.1f}, touchdown = {best['touchdown_angle_deg']:.1f}°, ",
            f"J = {best['objective']:.4f}, peak force = {best['peak_ground_force_body_weights']:.3f} body weights, ",
            f"dimensionless time to peak force = {best['dimensionless_time_to_peak_ground_force']:.3f}, stable = {best['stable']}, progress criterion passed = {best['progress_ok']}.",
            f"Measured mean touchdown angle: {best['mean_actual_touchdown_angle_deg']:.1f}° over {best['actual_touchdown_event_count']} contact events.",
            f"Termination: `{best['termination_reason']}`.",
            "Check whether the final best lies on the edge of its tested range; an edge result means the search range should be extended before claiming an optimum.",
            f"Heatmap: `{name}_objective_heatmap.png`; all coarse and refined trials: `{name}_trials.csv`.",
            "",
        ])
    (OUT / "best_results.json").write_text(json.dumps(best_results, indent=2))
    for filename, rows in (("weight_sensitivity.csv", sensitivity_rows),
                           ("pareto_frontier.csv", pareto_rows)):
        with (OUT / filename).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    (OUT / "summary.md").write_text("\n".join(report))
    print(f"\nSaved study outputs under: {OUT}")


if __name__ == "__main__":
    main()

