"""Dimensionless stiffness/touchdown-angle sweep for a humanoid gait simulator.

The simulator passed to grid_search must return these physical metrics:
positive_work_j, peak_ground_force_n, stride_time_s, stable, and
spring_return_ratio. The current view_humanoid_model.py is only a viewer;
humanoid_gait_sim.py must provide the actual walking/running controller.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math
from typing import Callable


@dataclass(frozen=True)
class DimensionlessCase:
    kappa: float                 # k L / (m g)
    touchdown_angle_rad: float


@dataclass(frozen=True)
class ObjectiveWeights:
    energy: float = 0.45
    peak_force: float = 0.20
    timing: float = 0.20
    spring_return: float = 0.15
    unstable_penalty: float = 100.0


def nondimensional_values(k_n_m: float, time_s: float, force_n: float,
                          mass_kg: float, leg_length_m: float,
                          gravity_m_s2: float = 9.81) -> dict[str, float]:
    """Convert SI stiffness, time, and force into project dimensionless values."""
    if mass_kg <= 0 or leg_length_m <= 0 or gravity_m_s2 <= 0:
        raise ValueError("mass, leg length, and gravity must be positive")
    return {
        "kappa": k_n_m * leg_length_m / (mass_kg * gravity_m_s2),
        "time_star": time_s * math.sqrt(gravity_m_s2 / leg_length_m),
        "force_star": force_n / (mass_kg * gravity_m_s2),
    }


def objective(metrics: dict[str, float | bool], *, mass_kg: float,
              leg_length_m: float, gravity_m_s2: float,
              target_stride_time_star: float,
              weights: ObjectiveWeights = ObjectiveWeights()) -> float:
    """Weighted score; smaller is better, and unstable trials are rejected."""
    if not metrics["stable"]:
        return weights.unstable_penalty

    energy_star = float(metrics["positive_work_j"]) / (
        mass_kg * gravity_m_s2 * leg_length_m)
    force_star = float(metrics["peak_ground_force_n"]) / (
        mass_kg * gravity_m_s2)
    time_star = float(metrics["stride_time_s"]) * math.sqrt(
        gravity_m_s2 / leg_length_m)
    timing_error = abs(time_star - target_stride_time_star) / max(
        target_stride_time_star, 1e-9)
    return_loss = 1.0 - min(1.0, max(0.0, float(metrics["spring_return_ratio"])))

    return (
        weights.energy * energy_star
        + weights.peak_force * force_star
        + weights.timing * timing_error
        + weights.spring_return * return_loss
    )


def grid_search(simulate_case: Callable, *, gait: str, mass_kg: float,
                leg_length_m: float, gravity_m_s2: float,
                target_stride_time_s: float, kappas: list[float],
                touchdown_angles_deg: list[float],
                weights: ObjectiveWeights = ObjectiveWeights(),
                output_csv: Path | None = None) -> list[dict]:
    """Evaluate every stiffness/angle pair using one fixed gait controller.

    `simulate_case(gait, kappa, touchdown_angle_rad)` must run the humanoid
    simulation and return the physical metrics documented at the top of file.
    """
    target_time_star = target_stride_time_s * math.sqrt(
        gravity_m_s2 / leg_length_m)
    trials = []

    for kappa in kappas:
        for angle_deg in touchdown_angles_deg:
            angle_rad = math.radians(angle_deg)
            metrics = simulate_case(gait, kappa, angle_rad)
            score = objective(
                metrics,
                mass_kg=mass_kg,
                leg_length_m=leg_length_m,
                gravity_m_s2=gravity_m_s2,
                target_stride_time_star=target_time_star,
                weights=weights,
            )
            trials.append({
                "gait": gait,
                "kappa": kappa,
                "touchdown_angle_deg": angle_deg,
                "touchdown_angle_rad": angle_rad,
                "positive_work_j": metrics["positive_work_j"],
                "energy_star": metrics["positive_work_j"] / (
                    mass_kg * gravity_m_s2 * leg_length_m),
                "peak_ground_force_n": metrics["peak_ground_force_n"],
                "peak_force_star": metrics["peak_ground_force_n"] / (
                    mass_kg * gravity_m_s2),
                "stride_time_s": metrics["stride_time_s"],
                "stride_time_star": metrics["stride_time_s"] * math.sqrt(
                    gravity_m_s2 / leg_length_m),
                "spring_return_ratio": metrics["spring_return_ratio"],
                "stable": metrics["stable"],
                "objective": score,
            })
            print(f"{gait}: kappa={kappa:.2f}, touchdown={angle_deg:.1f} deg, "
                  f"J={score:.4f}, stable={metrics['stable']}")

    trials.sort(key=lambda row: row["objective"])
    if output_csv is not None:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        with output_csv.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(trials[0]))
            writer.writeheader()
            writer.writerows(trials)
    return trials
