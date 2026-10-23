# MAE 598: Bio-Inspired Passive Energy Recovery (MuJoCo starter)

This starter project creates a reduced-order planar biped for comparing compliant walking and running. Each leg has a hip hinge and an axial prismatic spring-damper. Walking uses longer, overlapping stance schedules (inverted-pendulum-like support); running uses shorter stance schedules and a swing interval (SLIP-like rebound). During stance, a MuJoCo equality constraint approximates a planted foot; the code separately measures actual foot-ground contacts. The same spring/damper coordinates let the team compare gait-specific optima.

## Research question

How do optimal dimensionless leg stiffness and damping ratio differ between bipedal walking and running, and can one shared configuration provide efficient, stable motion in both?

## Model and equations

Each axial leg element is represented by

`F_s = -k (l - l_0) - c l_dot`

with stored elastic energy `E_s = 0.5 k (l-l_0)^2` and damper loss rate `P_d = c l_dot^2`.

The search variables are dimensionless stiffness `kappa = k L/(m g)` and damping ratio `zeta = c/(2 sqrt(k m_eff))`. The code uses `m=10.66 kg`, `L=0.74 m`, and `m_eff=m/2` as initial modeling assumptions. Update these in `gait_sim.py` when the team selects its final robot scale and parameter definitions.

Performance outputs include positive and negative actuator work, cost of transport, spring loading/release, damping loss, forward-speed error, center-of-mass height deviation, actual single/double support and flight fractions, and a simple fall/pitch stability flag. The reduced model includes trunk-height/pitch stabilization and swing-leg retraction actuators; all their work is included. Interpret these as comparative simulation metrics, not as validated hardware performance.

## Run it

```bash
python -m venv .venv
source .venv/bin/activate       # Windows: .venv\\Scripts\\activate
python -m pip install -r requirements.txt
python run_experiments.py       # baseline walk and run
python plot_results.py          # baseline comparison figures
python optimize.py              # Powell search per gait
python evaluate_shared.py       # compare shared vs gait-specific settings
```

Results are written to `results/` as CSV and JSON. To view a MuJoCo window, use the MuJoCo Python viewer or add a visualization script after the baseline runs.

## Important modeling caveat

This is a starter template, not a finished validated biped. The gait schedule is prescribed; there is no autonomous walk-run transition controller. The model uses a simplified trunk and two telescoping legs rather than a full multi-link humanoid. Before presenting results, verify that measured support and flight fractions match the intended gait, then tune mass, height, friction, speed, stride period, and controller gains so each baseline completes repeatable cycles. Use identical initial conditions and evaluation windows across parameter candidates. Report gait-specific optima and a common-parameter comparison separately.

## Suggested experiment sequence

1. Validate geometry, contact, and sign conventions with a short standing/drop test.
2. Establish repeatable baseline walking and running with conservative stiffness/damping.
3. Sweep a coarse grid in `(kappa, zeta)` and discard unstable trials.
4. Use Powell only inside a feasible region discovered by the sweep; report robustness with multiple initial conditions.
5. Compare each gait's own best parameters against one shared parameter pair using the same objective and constraints.
6. Plot cost of transport, spring return ratio, damping loss, and stability versus parameters.

## Files

- `biped.xml`: MuJoCo MJCF for the sagittal reduced-order biped.
- `gait_sim.py`: model loading, gait references, data collection, and metrics.
- `run_experiments.py`: baseline comparison.
- `optimize.py`: separate bounded Powell searches for walking and running.
