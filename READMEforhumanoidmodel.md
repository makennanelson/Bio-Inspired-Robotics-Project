# Optional Humanoid-v5 Animation Experiment

This folder's Humanoid-v5 PPO experiment is separate from the primary articulated modified-SLIP model and its energy/stability results. Use it as an optional visual comparison only.

This project uses MuJoCo to compare walking and running in a simplified, planar biped robot. We’re looking at how leg spring stiffness and damping affect energy use and stability in each gait.

The same robot model is used for both tests, but walking and running are simulated as separate, pre-set gaits. The model does not switch between them automatically.

## Project question

How do spring stiffness and damping affect energy use and stability during walking and running? Can one set of spring and damping values work well for both gaits, or does each gait need its own settings?

## The model

Each leg has a hip hinge, a knee hinge, an ankle hinge, and an axial telescoping spring/damper. The existing gait controller actuates the hip. The knee and ankle have passive torsional springs and dampers; during running, a knee motor bends the shank toward the robot’s forward (+x) direction during swing and extends it before touchdown. The ankle remains passive. The axial spring and damper apply the force:

`F_s = -k (l - l_0) - c l_dot`

The spring can store energy as it compresses and release some of it as the leg extends. The damper represents energy lost during that motion:

`E_s = 0.5 k (l - l_0)^2`

`P_d = c l_dot^2`

Walking is modeled with longer, overlapping stance phases. Running has shorter stance phases and periods when both feet are off the ground. A MuJoCo constraint approximates a planted foot during stance, while the simulation separately tracks actual foot-ground contacts. The model remains reduced-order: torso translation, pitch stabilization, and forward drive still use root-level actuators. The knees are motor-controlled during running; the ankles remain passive.

We use dimensionless stiffness `kappa = kL/(mg)` and damping ratio `zeta = c/(2 sqrt(k m_eff))` to compare settings. The current model uses `m = 10.66 kg`, `L = 0.74 m`, and `m_eff = m/2` as initial assumptions. These can be changed in `gait_sim.py` if the team chooses different model values.

## What we're measuring

The simulation records actuator work (including the running knee motors), cost of transport, axial and knee/ankle spring energy and return, damping loss, knee and ankle angles, speed error, center-of-mass height, foot support and flight time, and a basic stability flag. The original `spring_return_ratio` is for the axial leg springs; `total_spring_return_ratio` includes the passive knee and ankle springs. Existing results were generated with the rigid-leg model, so rerun the simulations and parameter searches before interpreting results for this articulated model.

These measures let us compare settings within the simulation. They should not be treated as proof of how a physical robot would perform.

## Running the simulation

```bash
python -m venv .venv
source .venv/bin/activate       # Windows: .venv\Scripts\activate
python -m pip install -r requirements.txt
python run_experiments.py       # run the baseline walking and running tests
python plot_results.py          # create baseline comparison plots
python optimize.py              # search for settings for each gait
python evaluate_shared.py       # compare shared and gait-specific settings
python sweep_parameters.py      # run the parameter sweep
python plot_sweep.py            # make plots from the saved sweep results
```

To view the articulated robot, run one gait at a time. On macOS, use `mjpython`:

```bash
mjpython view_gait.py --gait walk
mjpython view_gait.py --gait run
```

The hip joints are actuated by the existing gait controller. During running, the knee motors flex each shank toward the robot’s forward (+x) direction during swing and extend it before touchdown; during walking those motors remain off. The ankles remain passive. The viewer uses the same simulation controller and waits for you to close its window before returning to the terminal.

## Preview the built-in human-shaped model

The project also includes a viewer for Gymnasium's Humanoid-v5 model. It opens the human-shaped model in its initial pose; this is a visual/model preview and does not yet run the project's walking or running controller.

```bash
python -m pip install -r requirements.txt
mjpython view_humanoid_model.py
```

The humanoid is a separate model from `biped.xml`. To use it for the energy and stability comparison, the gait controller, spring model, and metric collection still need to be adapted to the humanoid's joints and contacts. Treat results from the two models as separate until that adaptation and the parameter sweeps are complete.
