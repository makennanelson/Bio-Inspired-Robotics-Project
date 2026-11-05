# Spring stiffness and touchdown angle search

The study tests dimensionless leg stiffness κ = kL/(mg) and the forward touchdown angle.
Touchdown angle is the swing controller's target; the simulation separately measures the actual world-frame leg angle at foot-contact transitions.
Damping ratio is held at 0.20, and the rest of each gait controller is fixed.
Peak force is the maximum summed normal force from the two feet, divided by body weight.
Dimensionless event time is τ = t_peak / sqrt(L/g) = t_peak * sqrt(g/L), with L = 0.74 m.
The optimized timing event is when the maximum total foot force occurs; spring-compression peak timing is reported separately.
The timing term penalizes relative deviation from baseline peak-force timing rather than rewarding an earlier impact.
Candidates must also complete the simulation and travel at least half the target distance; otherwise they receive a large penalty.
Objective: J = 0.75 × (candidate peak force / baseline peak force) + 
0.25 × relative change in dimensionless peak-force time; unstable trials add 100.

Each gait is searched independently on a 7 × 7 coarse grid, then two local 5 × 5 refinements around the best candidates.

### Weight sensitivity and trade-offs (walk)

| Force / timing weights | Best κ | Target touchdown | Measured touchdown | Peak force (BW) | Dimensionless peak time |
|---|---:|---:|---:|---:|---:|
| 100% / 0% | 5.00 | 5.00° | -0.1° | 14.32 | 20.273 |
| 75% / 25% | 2.00 | 25.00° | 16.2° | 14.37 | 21.824 |
| 50% / 50% | 2.00 | 25.00° | 16.2° | 14.37 | 21.824 |
| 25% / 75% | 25.00 | 25.00° | 16.6° | 14.56 | 21.664 |
| 0% / 100% | 25.00 | 20.00° | 12.7° | 19.72 | 21.635 |
Pareto frontier size: 9 feasible settings; see `pareto_frontier.csv` for non-dominated force/timing trade-offs.

## Walk

Baseline: peak force 17.522 body weights; 
dimensionless time to peak force 21.635.
Best grid point: κ = 2.0, touchdown = 25.0°, 
J = 0.6173, peak force = 14.370 body weights, 
dimensionless time to peak force = 21.824, stable = True, progress criterion passed = True.
Measured mean touchdown angle: 16.2° over 19 contact events.
Termination: `completed_duration`.
Check whether the final best lies on the edge of its tested range; an edge result means the search range should be extended before claiming an optimum.
Heatmap: `walk_objective_heatmap.png`; all coarse and refined trials: `walk_trials.csv`.

### Weight sensitivity and trade-offs (run)

| Force / timing weights | Best κ | Target touchdown | Measured touchdown | Peak force (BW) | Dimensionless peak time |
|---|---:|---:|---:|---:|---:|
| 100% / 0% | 7.50 | 27.50° | 16.9° | 7.64 | 14.586 |
| 75% / 25% | 7.50 | 25.00° | 15.7° | 7.68 | 2.112 |
| 50% / 50% | 7.50 | 22.50° | 14.4° | 8.16 | 2.003 |
| 25% / 75% | 7.50 | 22.50° | 14.4° | 8.16 | 2.003 |
| 0% / 100% | 20.00 | 20.00° | 4.6° | 22.35 | 1.966 |
Pareto frontier size: 10 feasible settings; see `pareto_frontier.csv` for non-dominated force/timing trade-offs.

## Run

Baseline: peak force 21.196 body weights; 
dimensionless time to peak force 1.966.
Best grid point: κ = 7.5, touchdown = 25.0°, 
J = 0.2904, peak force = 7.683 body weights, 
dimensionless time to peak force = 2.112, stable = True, progress criterion passed = True.
Measured mean touchdown angle: 15.7° over 32 contact events.
Termination: `completed_duration`.
Check whether the final best lies on the edge of its tested range; an edge result means the search range should be extended before claiming an optimum.
Heatmap: `run_objective_heatmap.png`; all coarse and refined trials: `run_trials.csv`.
