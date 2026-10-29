# Bio-Inspired Walking and Running Simulation

This project uses MuJoCo to compare walking and running in a human-shaped, articulated biped built around SLIP-style compliant legs. The legs have hip, knee, and ankle joints, and axial spring-dampers. Walking and running use separate prescribed gait schedules; the model does not autonomously switch between them.

## Project question

How do leg-spring stiffness and damping affect energy use and stability during walking and running? Can one set of dimensionless spring parameters work well for both gaits?

## Model and SLIP assumptions

The torso, pelvis, thighs, shanks, and feet give the model an articulated human-like shape. Each leg has an axial spring-damper, passive torsional knee and ankle springs, and hip control. During running, knee motors flex the leg during swing and extend it before touchdown.

The axial spring force acts along the leg's compression axis. With leg length L, rest length L0, and length rate Ldot, its magnitude is:

F_s = k (L0 - L) - c Ldot

The corresponding spring energy and damper loss rate are:

E_s = 0.5 k (L0 - L)^2
P_d = c Ldot^2

The walking schedule uses a duty factor of 0.62, which creates overlapping support. The running schedule uses a duty factor of 0.45 with the legs offset by half a cycle, leaving a nominal 10% aerial interval. During scheduled flight, direct root-level forward drive, vertical support, and pitch stabilization are set to zero. Root translation has no joint damping, so gravity drives the torso through the aerial phase when the feet are actually off the ground. Hip and knee swing motors can still move the limbs internally. During stance, the model retains root-level forward, vertical, and pitch assistance; their actuator work is included in the energy accounting. This is an actuated articulated modified SLIP model, not a fully passive SLIP system.

Foot-ground contact is measured separately from the prescribed stance schedule. Results therefore report both actual contact-based flight fraction and scheduled flight fraction. The running schedule should create flight, but the actual simulated robot must also lose ground contact for the run to exhibit an aerial phase.

Spring parameters use dimensionless stiffness kappa = k L/(m g) and damping ratio zeta = c/(2 sqrt(k m_eff)). The initial assumptions are m = 10.66 kg, L = 0.74 m, and m_eff = m/2; they are defined in gait_sim.py.

## Measurements and existing results

The simulation records actuator work, cost of transport, axial and joint spring loading/release, damping loss, joint angles, speed error, center-of-mass height, support fractions, actual flight, and scheduled flight. Spring energy storage and release are reported separately from motor work; this distinction matters because motors can add energy to the model.

The CSV files already in results/ are preserved as historical results from the earlier settings. New baseline runs, optimizations, and sweeps write to results/modified_slip/. Re-run those analyses before using results to make claims about this modified model.

These results describe the simulation and do not establish how a physical robot would perform.

## Run the analyses

python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python run_experiments.py

To view a gait, run one at a time. On macOS, use mjpython:

    mjpython view_gait.py --gait walk
    mjpython view_gait.py --gait run

The Humanoid-v5 PPO files are an optional, separate animation experiment. They are not the model used for the modified-SLIP energy and stability analyses.
