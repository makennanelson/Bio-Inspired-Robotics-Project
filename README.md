# Bio-Inspired Walking and Running Simulation

This project simulates a human-shaped, articulated planar biped using a modified SLIP-style model. It compares walking and running while varying the stiffness of the ground relative to the leg spring.

The model uses the same leg-spring parameters for both gaits. Walking and running have separate prescribed gait schedules; the robot does not switch between them automatically.

## Project question

How does ground stiffness affect energy use and foot-ground contact during walking and running when the leg-spring properties are held constant?

## Model and assumptions

The torso, pelvis, thighs, shanks, and feet give the model an articulated, human-like shape. Each leg has an axial spring-damper, passive torsional knee and ankle springs, and hip control. During running, knee motors flex the leg during swing and extend it before touchdown.

The axial spring force acts along the leg’s compression axis. With leg length \(L\), rest length \(L_0\), and length rate \(\dot{L}\), its magnitude is:

\[
F_s = k(L_0 - L) - c\dot{L}
\]

The corresponding spring energy and damper loss rate are:

\[
E_s = \frac{1}{2}k(L_0 - L)^2
\]

\[
P_d = c\dot{L}^2
\]

The model represents leg and ground compliance as two linear springs in series:

\[
\frac{1}{k_{\mathrm{eff}}} =
\frac{1}{k_{\mathrm{leg}}} +
\frac{1}{k_{\mathrm{ground}}}
\]

This is a reduced-order approximation: the floor itself does not deform in the simulation. The dimensionless leg and ground stiffnesses are defined as:

\[
\kappa = \frac{kL}{mg}
\]

The simulation uses a mass of 10.66 kg and a leg length of 0.74 m.

The walking schedule uses a duty factor of 0.62, creating overlapping support. The running schedule uses a duty factor of 0.38, with the legs offset by half a cycle. This schedules about 24% of the cycle with neither leg in stance. The actual robot must also lose foot-ground contact for that time to count as simulated flight.

During scheduled flight, direct root-level forward drive, vertical support, and pitch stabilization are set to zero. The root has no joint damping, so gravity drives the torso during flight. Hip and knee motors can still reposition the legs. During stance, the model retains root-level forward, vertical, and pitch assistance, and their work is included in the energy accounting. This is an actuated modified-SLIP model, not a fully passive SLIP system.

## Measurements and current results

The simulation records actuator work, cost of transport, axial and joint spring loading and release, damping loss, joint angles, speed error, center-of-mass height, support fractions, actual flight, and scheduled flight. Spring energy storage and release are reported separately from motor work because motors can add energy to the model.

The ground-stiffness sweep tests both walking and running at dimensionless ground stiffness values of 10, 25, 50, 100, and 200. It holds dimensionless leg stiffness at 15 and damping ratio at 0.20.

In the current saved results, all ten simulations completed the code’s numerical and safety checks. Walking’s measured flight fraction is approximately zero, as expected for its schedule. For running, measured flight is about 0.276–0.291, compared with about 0.233 scheduled. The measured-versus-scheduled difference is greater than 0.05 at ground stiffness values 10 and 25, and less than 0.05 at 50, 100, and 200.

The `stable` field means the simulation completed without triggering its numerical and safety guards. It does not establish formal periodic-gait stability. The results describe this simulation and do not establish how a physical robot would perform.

## Run the analyses

On macOS, from the project folder:

```bash
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python run_experiments.py
python sweep_ground_stiffness.py
python analyze_ground_stiffness.py 

## View one gait at a time

Run one command at a time:


```bash
mjpython view_gait.py --gait walk
```

```bash
mjpython view_gait.py --gait run
```

## Open the ground-stiffness CSV in Microsoft Excel (macOS)

```bash
open -a "Microsoft Excel" "results/ground_stiffness/ground_stiffness_summary.csv"
```