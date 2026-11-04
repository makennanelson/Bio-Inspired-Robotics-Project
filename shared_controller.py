"""Build one speed- and ground-stiffness-aware gait controller."""
from dataclasses import replace

from gait_sim import GAITS, GaitConfig, SpringParameters, smoothstep


def make_shared_gait(
    target_speed: float,
    dimensionless_ground_stiffness: float,
) -> tuple[GaitConfig, SpringParameters]:
    """Blend walking-to-running control settings using speed and ground stiffness."""
    walk = GAITS["walk"]
    run = GAITS["run"]

    # Blend smoothly from walking at 0.85 m/s to running at 2.0 m/s.
    transition_start = 0.9
    transition_end = 1.40
    blend = smoothstep(
        (target_speed - transition_start) / (transition_end - transition_start)
    )

    # The existing sweep suggests softer ground needs slightly more scheduled
    # flight. This is an initial calibration to test, not a proven optimum.
    kappa = max(10.0, min(200.0, dimensionless_ground_stiffness))
    run_duty = 0.354 + (kappa - 10.0) * (0.402 - 0.354) / 190.0

    duty_factor = (1.0 - blend) * walk.duty_factor + blend * run_duty
    cadence_blend = smoothstep(
        (target_speed - walk.speed) / (run.speed - walk.speed)
    )
    period = (1.0 - cadence_blend) * walk.period + cadence_blend * run.period
    stride_length = target_speed * period

    gait = replace(
        walk,
        name="shared",
        speed=target_speed,
        stride_length=stride_length,
        duty_factor=duty_factor,
        drive_kp=(1.0 - blend) * walk.drive_kp + blend * run.drive_kp,
        drive_ki=(1.0 - blend) * walk.drive_ki + blend * run.drive_ki,
        run_blend=blend,
    )

    params = SpringParameters(
        dimensionless_stiffness=15.0,
        damping_ratio=0.20,
        dimensionless_ground_stiffness=dimensionless_ground_stiffness,
    )
    return gait, params