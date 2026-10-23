# Bio-Inspired Walking and Running Simulation

This project uses MuJoCo to compare walking and running in a simplified, planar biped robot. We’re looking at how leg spring stiffness and damping affect energy use and stability in each gait.

The same robot model is used for both tests, but walking and running are simulated as separate, pre-set gaits. The model does not switch between them automatically.

## Project question

How do spring stiffness and damping affect energy use and stability during walking and running? Can one set of spring and damping values work well for both gaits, or does each gait need its own settings?

## The model

Each leg has a spring and damper that apply the force:

`F_s = -k (l - l_0) - c l_dot`

The spring can store energy as it compresses and release some of it as the leg extends. The damper represents energy lost during that motion:

`E_s = 0.5 k (l - l_0)^2`

`P_d = c l_dot^2`

Walking is modeled with longer, overlapping stance phases. Running has shorter stance phases and periods when both feet are off the ground. A MuJoCo constraint approximates a planted foot during stance, while the simulation separately tracks actual foot-ground contacts.

We use dimensionless stiffness `kappa = kL/(mg)` and damping ratio `zeta = c/(2 sqrt(k m_eff))` to compare settings. The current model uses `m = 10.66 kg`, `L = 0.74 m`, and `m_eff = m/2` as initial assumptions. These can be changed in `gait_sim.py` if the team chooses different model values.

## What we measure

The simulation records actuator work, cost of transport, spring energy and return, damping loss, speed error, center-of-mass height, foot support and flight time, and a basic stability flag.

These measures let us compare settings within the simulation. They should not be treated as proof of how a physical robot would perform.

## Run the simulation

```bash
python -m venv .venv
source .venv/bin/activate       # Windows: .venv\Scripts\activate
python -m pip install -r requirements.txt
python run_experiments.py       # run the baseline walking and running tests
python plot_results.py          # create baseline comparison plots
python optimize.py              # search for settings for each gait
python evaluate_shared.py       # compare shared and gait-specific settings
python sweep_compare.py         # run the duty-factor-0.60 sweep and make plots
