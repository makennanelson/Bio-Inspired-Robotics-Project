from dataclasses import replace
from pathlib import Path

from gait_sim import GAITS, SpringParameters, run_simulation

output_dir = Path("results/ground_stiffness/schedule_tuning")
output_dir.mkdir(parents=True, exist_ok=True)

params = SpringParameters(
    dimensionless_stiffness=15.0,
    damping_ratio=0.20,
    dimensionless_ground_stiffness=50.0,
)

for duty_factor in (0.38, 0.40, 0.42, 0.44, 0.46):
    gait = replace(GAITS["run"], duty_factor=duty_factor)
    output_csv = output_dir / f"run_duty_{duty_factor:.2f}.csv"
    result = run_simulation(gait, params, output_csv)

    print(
        f"duty={duty_factor:.2f}  "
        f"scheduled_flight={result['scheduled_flight_fraction']:.3f}  "
        f"actual_flight={result['flight_fraction']:.3f}  "
        f"gap={abs(result['flight_fraction_difference']):.3f}  "
        f"speed_error={result['rms_speed_error_m_s']:.3f} m/s  "
        f"completed={result['stable']}"
    )