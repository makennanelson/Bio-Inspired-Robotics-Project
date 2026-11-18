# Bio-Inspired Walking and Running Simulation

This project uses MuJoCo to simulate a two-legged robot. The robot has a human-like shape and uses springs in its legs. We compare walking and running while changing the simulated ground stiffness.

Walking and running are tested as separate gaits. The robot does not switch between them on its own. In the ground-stiffness tests, both gaits use the same leg-spring stiffness and damping.

## Project question

How does ground stiffness affect energy use and foot contact during walking and running when leg stiffness and damping stay the same?

## Model

The model has a torso, pelvis, thighs, shins, and feet. Each leg has a spring and damper. The knees and ankles have passive springs, and motors control the hips. During running, knee motors also move the legs during each step.

This is a **modified SLIP model**, not a fully passive robot. During stance, motors help the torso move forward, stay at the right height, and control its pitch. These torso motors turn off during planned flight, but the hip and knee motors can still move the legs. The project uses a simulation, not a physical robot.

The model uses a mass of 10.66 kg, a leg length of 0.74 m, and gravity of 9.81 m/s². The floor stays rigid. We represent ground softness with a spring connected in series with the leg spring:

```text
1 / k_effective = 1 / k_leg + 1 / k_ground
```

This is a simplified way to model ground softness. The floor itself does not bend.

## Dimensionless stiffness and time

We use dimensionless values to compare stiffness and time relative to the robot’s size, weight, and gravity:

```text
κ = k L / (m g)
τ = t √(g / L)
```

Here, `k` is spring stiffness, `L` is leg length, `m` is robot mass, `g` is gravity, and `t` is time in seconds. A larger `κ` means the spring is stiffer compared with the robot’s weight and leg length. `τ` measures time using a scale based on gravity and leg length. The same dimensionless value can mean a different spring stiffness or time in seconds for a robot with a different mass, leg length, or gravity.

The model calculates spring force, stored spring energy, and energy lost through damping:

```text
Spring force magnitude: F_s = k (L_0 - L) - c L_dot
Spring energy:          E_s = 1/2 k (L_0 - L)^2
Damper loss rate:       P_d = c L_dot^2
```

`L` is the current leg length, `L_0` is the resting leg length, `L_dot` is how quickly the leg length changes, and `c` is damping.

## Walking and running schedules

| Gait | Target speed | Stride length | Duty factor | Schedule |
|---|---:|---:|---:|---|
| Walk | 0.85 m/s | 0.72 m | 0.62 | The legs overlap in support; no flight is planned. |
| Run in the current code | 2.00 m/s | 1.05 m | 0.48 | The legs are half a cycle apart; about 4% of the cycle has no planned support. |

The duty factor is the part of a leg’s cycle when it is scheduled to support the robot. Planned flight does not always mean the feet actually leave the ground. MuJoCo measures actual foot contact separately.

## What we measure

The simulation records motor work, cost of transport, spring energy, energy lost through damping, joint angles, speed error, torso height, foot contact, support time, and planned and actual flight time. Spring energy is reported separately from motor work because the motors can add energy to the robot.

The `stable` result only means that the run finished without crossing the limits set in the code. It does not prove that the robot can repeat the gait or recover after a push.

## Ground-stiffness tests and saved results

The ground-stiffness script tests walking and running with ground stiffness values of 10, 25, 50, 100, and 200. Leg stiffness stays at `κ_leg = 15`, and damping stays at `ζ = 0.20`. This gives ten tests in total.

**The saved results and the current code use different running schedules.** The CSV files included in this project were made with a running duty factor of 0.38. The current `gait_sim.py` file uses 0.48. A duty factor of 0.38 plans about 24% flight, while 0.48 plans about 4%.

In the saved running results, planned flight is about 23.3% and actual flight is about 27.6–29.1%. The difference between planned and actual flight is over 0.05 at ground stiffness values 10 and 25. It is below 0.05 at values 50, 100, and 200.

All ten saved tests finished without triggering the code’s safety checks. These results match the 0.38 running schedule, not a new run of the current code with a duty factor of 0.48. Running the sweep again will replace the CSV files with results from the current settings. Save a copy of the existing CSV files first if you want to keep the older results.

These results apply only to this model and controller. They do not show how a person or a physical robot would perform.

## Set up and run the simulations

These commands are for macOS. Run them from the project folder:

```bash
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Run the baseline walking and running tests:

```bash
python run_experiments.py
```

Run the ground-stiffness tests:

```bash
python sweep_ground_stiffness.py
```

Create plots and flight-contact checks from the sweep:

```bash
python analyze_ground_stiffness.py
```

The sweep saves its CSV files in `results/ground_stiffness/`. The analysis script saves plots and a flight-check CSV in `results/ground_stiffness/analysis/`.

## View the robot

On macOS, use `mjpython` to open the MuJoCo viewer. Run one command at a time:

```bash
mjpython view_gait.py --gait walk
```

```bash
mjpython view_gait.py --gait run
```

Close the MuJoCo window to return to the terminal. A viewer run is separate from the ground-stiffness tests.

## Open the summary in Excel

On macOS, open the saved summary file with:

```bash
open -a "Microsoft Excel" "results/ground_stiffness/ground_stiffness_summary.csv"
```
