A MATLAB/Simulink project for studying how a quadrotor drone responds to
horizontal wind shear and downdrafts. The project uses cascaded PID
control for horizontal position, altitude, and pitch attitude. A Python
implementation was used to explore and tune the controller equations;
the Simulink model should be run in your own MATLAB installation to
verify the final results.
Important: This is a simplified educational model, not a
flight-certified controller. Validate the model and tune parameters
against your actual airframe before using any results for real-world
flight.

Contents
- Project objectives
- Model overview
- Mathematical model
- PID controller equations
- Wind-shear model
- Simulation scenarios
- Reported study results
- Requirements and execution
- Repository structure
- Images and plots
- Limitations and future work
Project objectives
1. Model simplified quadrotor motion in the horizontal plane and
   vertical direction.
2. Simulate a time-varying wind gust and a height-dependent wind
   profile.
3. Use PID control to regulate position, altitude, and pitch.
4. Compare PID control against PD-only control.
5. Study the effect of wind intensity and position-loop gain selection.
6. Provide a base for future extensions such as full 6-DOF dynamics and
   disturbance-observer feedforward.
Model overview

The simplified drone model includes:
- Mass: 1.5 kg (representative starting value).
- States: horizontal position (x), horizontal velocity
  (\dot{x}{=tex}), altitude (z), vertical velocity
  (\dot{z}{=tex}), pitch angle (\theta{=tex}), and pitch rate
  (q=\dot{\theta}{=tex}).
- Disturbances: horizontal wind shear and a downdraft.
- Actuator approximation: first-order motor/thrust response with a
  nominal 30 ms time constant.
- Controller loops: horizontal position PID, altitude PID, and
  pitch-attitude PID.
The actual parameters are defined in drone_windshear_params.m. Check
that file before interpreting or changing the simulation.
Mathematical model
The following equations describe a common simplified planar
quadrotor. They are useful for understanding the project; the
implementation in the MATLAB files is the definitive source for the
exact signs, state definitions, and limits used by this repository.
1. Translational dynamics
Let:
- (x): horizontal position (m)
- (z): altitude, positive upward (m)
- (m): drone mass (kg)
- (T): total thrust (N)
- (\theta{=tex}): pitch angle (rad)
- (g): gravitational acceleration (approximately
  (9.81,\text{m/s}{=tex}^2))
- (w_x,w_z): horizontal and vertical wind velocities (m/s)
- (D_x,D_z): aerodynamic drag forces (N)
A simplified model is

\(m`\ddot{x}`{=tex}=-T`\sin`{=tex}`\theta`{=tex}+D_x\)

\(m`\ddot{z}`{=tex}=T`\cos`{=tex}`\theta`{=tex}-mg+D_z\)

The horizontal thrust sign depends on the pitch-angle convention. If the
project defines positive pitch in the opposite direction, the horizontal
equation's sign must be changed consistently in both plant and
controller.

A simple relative-wind drag model can be written as

\(D_x=-c_x(`\dot{x}`{=tex}-w_x)\)

\(D_z=-c_z(`\dot{z}`{=tex}-w_z)\)

where (c_x) and (c_z) are effective drag coefficients. This linear
approximation is convenient for control studies; real aerodynamic drag
is often nonlinear.
2. Pitch attitude dynamics
For a simplified single-axis pitch model,

\(`\dot{\theta}`{=tex}=q\)

\(I_y`\dot{q}`{=tex}=`\tau`{=tex}\_`\theta`{=tex}-c_q q\)

where (I_y) is the pitch-axis moment of inertia,
(\tau{=tex}_\theta{=tex}) is the pitch control torque, and (c_q)
represents rotational damping.
3. Hover thrust
For level hover with negligible vertical acceleration and drag,

\(T\_{`\text{hover}`{=tex}}=mg\)

For a 1.5 kg example,

\(T\_{`\text{hover}`{=tex}}=1.5(9.81)=14.715 `\text{N}`{=tex}\)

This is the total thrust from all rotors, not the thrust from one motor.
4. Motor/thrust lag
A first-order actuator model is

\(`\tau`{=tex}*m`\dot{T}`{=tex}+T=T*{`\text{cmd}`{=tex}}\)

where (\tau{=tex}m=0.03,\text{s}{=tex}) is the nominal response
time and (T{\text{cmd}{=tex}}) is the commanded thrust. If the
implementation models each motor separately, use its implemented motor
equations instead.
PID controller equations
A conventional PID controller is
e(`\lambda`{=tex}),d`\lambda`{=tex}+K_d`\frac{de(t)}{dt}`{=tex} \]

where (e(t)=r(t)-y(t)) is the reference-minus-measured error. In
discrete implementations, the derivative is commonly filtered to reduce
amplification of sensor noise, and the integral term should use
anti-windup when the output saturates.

### 1. Horizontal position loop

\[ e_x=x\_{`\text{ref}`{=tex}}-x \]

\[
`\theta`{=tex}*{`\text{cmd}`{=tex}}=`\operatorname{sat}`{=tex}`\left`{=tex}(
K*{px}e_x+K\_{ix}`\int `{=tex}e_x,dt+K\_{dx}`\dot{e}`{=tex}\_x
`\right`{=tex}) \]

The output is a commanded pitch angle. The saturation function limits
the requested tilt to a configured safe modelling limit.

### 2. Pitch-attitude loop

\[ e\_`\theta`{=tex}=`\theta`{=tex}\_{`\text{cmd}`{=tex}}-`\theta`{=tex}
K*{p`\theta`{=tex}}e\_`\theta`{=tex}+
K\_{i`\theta`{=tex}}`\int `{=tex}e\_`\theta`{=tex},dt+
K\_{d`\theta`{=tex}}`\dot{e}`{=tex}\_`\theta`{=tex} \]

The output is a pitch torque command or an equivalent control input to
the simplified attitude plant.

### 3. Altitude loop

\[ e_z=z\_{`\text{ref}`{=tex}}-z \]

\[ T\_{`\text{cmd}`{=tex}}=`\operatorname{sat}`{=tex}`\left[
mg+K_{pz}e_z+K_{iz}\int e_z\,dt+K_{dz}\dot{e}_z
\right]`{=tex}\]

The (mg) term is hover-thrust feedforward. Include it only if the MATLAB
implementation uses this convention; do not add it a second time if the
altitude controller already outputs total thrust.

### 4. Why integral action helps

A persistent wind disturbance can create a steady position offset when
the controller has only proportional and derivative terms. Integral
action accumulates the remaining error and can remove that offset,
provided the system is controllable and the integrator is managed
correctly. Integral action does not guarantee zero error in every
nonlinear or saturated situation.

## Wind-shear model

The study uses a gust front that raises the horizontal wind toward **8
m/s over approximately 8 seconds**, beginning at **10 seconds**,
together with a downdraft that can reach **3 m/s**. Wind speed also
increases with altitude.

A generic smooth gust profile can be represented as

\[ w_x(t,z)=w\_{`\text{base}`{=tex}}(z)+`\Delta `{=tex}w,S(t) \]

where (S(t)) is a smooth transition from 0 to 1. One possible smooth
step is

\[ S(t)=
```{=tex}
\begin{cases}
0, & t<t_0\\
3r^2-2r^3, & t_0\le t\le t_0+T_g\\
1, & t>t_0+T_g
\end{cases}
```
with
\[

\]
Here (t_0=10,\text{s}{=tex}) and (T_g=8,\text{s}{=tex}) are the
example gust start time and transition duration. This formula is an
explanatory example; use the exact wind function in the project code
when reproducing results.

A simplified downdraft can be written as

\(w_z(t)=-w\_{d,`\max`{=tex}}S_d(t)\)

where (w_{d,\max{=tex}}) is the maximum downdraft magnitude. The sign
assumes positive altitude is upward.
Simulation scenarios
The study script is intended to run three comparisons:
1. PID versus PD-only: examine whether integral action reduces
   persistent position offset.
2. Wind-shear intensity sweep: increase wind strength and monitor
   position error and pitch/tilt demand.
3. Position-gain comparison: assess the trade-off between reduced
   drift and increased overshoot.
Recommended plots to save from MATLAB/Simulink:
- Horizontal position and position reference versus time.
- Altitude and altitude reference versus time.
- Wind velocity versus time.
- Pitch angle versus time.
- Position error versus time.
- PID-versus-PD comparison.
- Peak error and peak tilt versus wind intensity.
Reported study results
The original Python-equation study reported the following values for the
8 m/s wind case. These values are preliminary study results, not
yet independently verified by running the Simulink model.

  Controller                Peak position error   Final position error   Peak tilt
  PID                                    0.49 m                    0 m        9.6°
  PD only (no integral)                  0.70 m                 0.70 m        9.5°

Additional observations reported by the study:
- Persistent wind can produce a steady offset with PD-only control.
- At 20 m/s wind, the required tilt was reported to reach about 23°,
  near the configured 25° limit.
- Aggressive position gains reduced drift but caused up to 54%
  calm-air step overshoot in the highest-gain case.
- The altitude error was reported to remain around 0.06 m in the
  tested scenario.
Reproduce and verify these figures by running the scripts in your
MATLAB release, checking the generated plots, and updating this table if
the results differ. The results depend on the exact model, initial
conditions, solver settings, limits, and parameter values.
Requirements and execution
Requirements
- MATLAB
- Simulink
- Project files from this repository
No additional MATLAB toolbox is intended to be required unless the
model-building script uses blocks from an optional toolbox in your
version.
Run the project
Open MATLAB and change directory to the project folder:
cd windshear_drone_project
build_drone_windshear_model
run_drone_windshear_study
The project uses drone_windshear_params.m for model parameters. Review
it before running a study.
Troubleshooting
- Block parameter error: PID block parameter names can differ
  between MATLAB/Simulink releases. Open the model-building script and
  check the parameter names accepted by your installed PID Controller
  block.
- Missing block/library: confirm that Simulink is installed and
  available on the MATLAB path.
- Results differ: check solver type and step size, initial
  conditions, saturation limits, wind profile, and parameter values.
- No figures are saved: inspect run_drone_windshear_study.m and
  add saveas or exportgraphics commands where the plots are
  created.
Repository structure
Example structure (adjust it to match the files you actually upload):
windshear_drone_project/
├── README.md
├── build_drone_windshear_model.m
├── run_drone_windshear_study.m
├── drone_windshear_params.m
└── images/
    ├── quadrotor_windshear_model.svg
    ├── pid_control_architecture.svg
    ├── position_response.png
    ├── altitude_response.png
    └── wind_profile.png
Only the first three MATLAB files and the two SVG diagrams are described
here as known project assets. Add the PNG plots after generating them;
do not leave references to plot files that are not actually present.
Images and plots
The two conceptual diagrams below are included in this repository:
- images/quadrotor_windshear_model.svg --- conceptual quadrotor and
  wind-shear illustration.
- images/pid_control_architecture.svg --- cascade PID control
  architecture.
After running the MATLAB study, export actual simulation plots into
images/ and add them to this README using:
![Horizontal position response](images/position_response.png)
![Altitude response](images/altitude_response.png)
![Wind profile](images/wind_profile.png)
Example MATLAB export:
% Run this after creating/selecting the relevant figure
exportgraphics(gcf, fullfile('images','position_response.png'), 'Resolution', 200);
Make sure the selected figure is the intended plot before exporting it.
Limitations and future work
- The model is simplified and does not represent full 6-DOF rigid-body
  dynamics.
- Real rotor aerodynamics, motor mixing, sensor noise, battery
  effects, and ground effect may not be included.
- Wind shear is an idealized disturbance rather than a validated
  atmospheric model.
- PID gains are representative starting values and must be tuned for
  the implemented plant.
- The reported Python results do not replace verification in
  MATLAB/Simulink.
Possible extensions:
1. Add full 6-DOF translational and rotational dynamics.
2. Include IMU/GNSS sensor noise and state estimation using a
   complementary filter or EKF.
3. Add gust models and stochastic turbulence.
4. Add disturbance-observer or wind-feedforward compensation.
5. Compare PID with LQR, MPC, or adaptive control.
6. Run gain optimization and Monte Carlo robustness tests.
