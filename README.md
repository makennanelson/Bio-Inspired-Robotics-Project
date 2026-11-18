Bio-Inspired Walking and Running Simulation
This project uses MuJoCo to simulate a human-shaped, two-legged robot. The model is based on the spring-loaded inverted pendulum (SLIP) idea. We use it to study walking and running while changing the stiffness of the simulated ground.

Walking and running use separate schedules. The robot does not change between them by itself. The same leg spring stiffness and damping are used for both gaits in the ground-stiffness sweep.
Project question
How does changing ground stiffness affect energy use and foot-ground contact during walking and running when leg stiffness and damping stay the same?
Model
The model has a torso, pelvis, thighs, shanks, and feet. Each leg has a spring and damper along its length, passive spring-like knee and ankle joints, and a controlled hip. During running, knee motors move the legs during swing and prepare them for landing.

This is a modified SLIP model. The torso has motors that help it move forward, rise, and control its pitch during stance. Those motors turn off during planned flight, but the hip and knee motors can still move the legs. The model is not a fully passive SLIP robot, and it is not a physical robot.

The model uses a mass of 10.66 kg, a nominal leg length of 0.74 m, and gravity of 9.81 m/s². The floor is rigid in the simulation. Ground softness is represented with a spring in series with the leg spring:
1 / k_effective = 1 / k_leg + 1 / k_ground
This spring model approximates ground compliance; the floor itself does not deform.
Dimensionless parameters
Dimensionless values let us describe stiffness and time relative to the robot’s size, weight, and gravity:
κ = k L / (m g)
τ = t √(g / L)
Here, k is spring stiffness, L is leg length, m is robot mass, g is gravity, and t is time in seconds. A larger κ means a stiffer spring compared with the robot’s weight and leg length. τ expresses time using the gravity-and-leg-length time scale. The same dimensionless value can correspond to different stiffnesses in N/m or times in seconds if the mass, leg length, or gravity changes.

The spring force, stored energy, and damper energy loss are calculated as:
Spring force magnitude: F_s = k (L_0 - L) - c L_dot
Spring energy:          E_s = 1/2 k (L_0 - L)^2
Damper loss rate:       P_d = c L_dot^2
L is the current leg length, L_0 is its resting length, L_dot is the change in leg length over time, and c is damping.
Gait schedules
Gait	Target speed	Stride length	Duty factor	Schedule
Walk	0.85 m/s	0.72 m	0.62	The legs have overlapping support; no planned flight.
Run in the current code	2.00 m/s	1.05 m	0.48	The legs are half a cycle apart; about 4% of the cycle has no scheduled stance.


The duty factor is the fraction of a leg’s cycle when that leg is scheduled to support the robot. Planned flight does not guarantee actual flight: MuJoCo separately measures whether the feet lose contact with the ground.
What the simulation measures
The simulation records motor work, cost of transport, spring energy stored and released, damping losses, joint angles, speed error, torso height, foot contacts, support time, and planned and measured flight time. Spring energy is reported separately from motor work because motors can add energy to the model.

The stable result means only that a run finished without crossing the code’s numerical and safety limits. It does not prove that the gait repeats reliably or is stable after a push.
Ground-stiffness tests and saved results
The ground-stiffness script tests both gaits at dimensionless ground stiffness values of 10, 25, 50, 100, and 200. It keeps leg stiffness at κ_leg = 15 and damping ratio at ζ = 0.20. This produces ten runs in total.

Important: the saved results and current code use different running schedules. The CSV files included in this ZIP were generated with running duty factor 0.38. The current gait_sim.py sets it to 0.48. At 0.38, the schedule plans about 24% flight; at 0.48, it plans about 4%. The archived CSVs show about 23.3% scheduled flight and about 27.6–29.1% measured flight during running. Their flight-schedule gap is over 0.05 at ground stiffness 10 and 25, and below 0.05 at 50, 100, and 200.

The ten archived tests finished without triggering the code’s numerical and safety checks. These archived results describe the 0.38 running schedule, not a fresh run of the current 0.48 code. If you run the sweep again, the CSV files will be replaced with results from the current code settings. Keep a copy of the archived files if you want to compare the two schedules.

The results apply to this model and controller. They do not show how a person or a physical robot would perform.
Setup and run the simulations
These commands are for macOS. Run them from the project folder:
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
Run the baseline walking and running tests:
python run_experiments.py
Run the ground-stiffness sweep:
python sweep_ground_stiffness.py
Analyze the sweep and make plots:
python analyze_ground_stiffness.py
The sweep writes its CSV files to results/ground_stiffness/. The analysis script saves plots and a flight-schedule diagnostic CSV in results/ground_stiffness/analysis/.
View a gait
On macOS, use mjpython to open the MuJoCo viewer. Run one gait at a time:
mjpython view_gait.py --gait walk
mjpython view_gait.py --gait run
Close the MuJoCo viewer to return to the terminal. A viewer run is separate from the ground-stiffness sweep.
Open the summary in Excel
On macOS, open the saved sweep summary with:
open -a "Microsoft Excel" "results/ground_stiffness/ground_stiffness_summary.csv"
