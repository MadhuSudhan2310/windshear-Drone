# Quadrotor Drone Wind Shear Simulation Using PID Control

![Quadrotor Drone](images/quadrotor_windshear_model.svg)

## 📌 Project Overview

This project focuses on the simulation of a quadrotor drone operating under wind shear and downdraft conditions using MATLAB and Simulink. A cascaded PID controller is designed to maintain horizontal position, altitude, and pitch attitude despite external wind disturbances.

The project also compares PID and PD controllers to evaluate position tracking, steady-state error, and drone stability.

## 🎯 Objectives

- Develop a simplified quadrotor drone dynamics model.
- Simulate horizontal wind shear and vertical downdrafts.
- Implement altitude, position, and pitch PID controllers.
- Compare PID and PD-only controller performance.
- Study the effects of wind intensity on drone stability.
- Analyze position error, altitude error, and pitch angle.
- Explore improvements for robust UAV flight control.

## 🛠️ Tools and Technologies

- MATLAB
- Simulink
- PID Control Systems
- Quadrotor Dynamics
- Wind Disturbance Modelling
- Python for preliminary equation-based testing

## 🚁 Drone Model Parameters

| Parameter | Value |
|---|---:|
| Drone mass | 1.5 kg |
| Maximum horizontal wind | 8 m/s |
| Maximum downdraft | 3 m/s |
| Wind gust start time | 10 s |
| Wind transition duration | 8 s |
| Motor response time | 30 ms |
| Configured maximum tilt | 25° |

These are representative study parameters. Refer to `drone_windshear_params.m` for the actual implementation values.

## 📐 Mathematical Equations

### 1. Horizontal Motion

The simplified horizontal dynamics are

\[
m\ddot{x}=-T\sin\theta+D_x
\]

where:

- \(m\) = drone mass (kg)
- \(x\) = horizontal position (m)
- \(T\) = total thrust (N)
- \(\theta\) = pitch angle (rad)
- \(D_x\) = horizontal aerodynamic drag (N)

A simple drag model is

\[
D_x=-c_x(\dot{x}-w_x)
\]

where \(c_x\) is an effective drag coefficient and \(w_x\) is horizontal wind velocity.

### 2. Vertical Motion

The altitude dynamics are

\[
m\ddot{z}=T\cos\theta-mg+D_z
\]

where \(z\) is altitude, \(g=9.81\,\text{m/s}^2\), and \(D_z\) is vertical aerodynamic drag.

For level hover,

\[
T_{\text{hover}}=mg
\]

For a 1.5 kg drone,

\[
T_{\text{hover}}=1.5\times9.81=14.715\text{ N}
\]

This represents the combined thrust of all four motors.

### 3. Pitch Dynamics

The simplified pitch-axis equations are

\[
\dot{\theta}=q
\]

\[
I_y\dot{q}=\tau_\theta-c_q q
\]

where \(I_y\) is pitch moment of inertia, \(q\) is pitch rate, and \(\tau_\theta\) is the control torque.

### 4. PID Controller

The general PID control equation is

\[
u(t)=K_p e(t)+K_i\int e(t)\,dt+K_d\frac{de(t)}{dt}
\]

where

- \(K_p\) = proportional gain
- \(K_i\) = integral gain
- \(K_d\) = derivative gain
- \(e(t)\) = reference minus measured output

### 5. Position PID

\[
e_x=x_{\text{ref}}-x
\]

\[
\theta_{\text{cmd}}=
K_{px}e_x+K_{ix}\int e_x\,dt+K_{dx}\dot{e}_x
\]

The commanded pitch angle is limited to the configured tilt range.

### 6. Altitude PID

\[
e_z=z_{\text{ref}}-z
\]

\[
T_{\text{cmd}}=mg+
K_{pz}e_z+
K_{iz}\int e_z\,dt+
K_{dz}\dot{e}_z
\]

The hover-thrust term is included when the controller outputs total thrust and the implementation uses this control convention.

### 7. Pitch Attitude PID

\[
e_\theta=\theta_{\text{cmd}}-\theta
\]

\[
\tau_{\theta,\text{cmd}}=
K_{p\theta}e_\theta+
K_{i\theta}\int e_\theta\,dt+
K_{d\theta}\dot{e}_\theta
\]

Derivative filtering and anti-windup help reduce noise sensitivity and integral accumulation during saturation.

## 🌬️ Wind Shear Model

Wind shear represents a change in wind velocity over time or altitude.

A simplified horizontal wind model is

\[
w_x(t,z)=w_{\text{base}}(z)+\Delta w S(t)
\]

where \(S(t)\) represents the gust transition.

For a smooth transition,

\[
S(t)=3r^2-2r^3
\]

with

\[
r=\frac{t-t_0}{T_g}
\]

during the gust transition interval.

The example gust begins at \(t_0=10\) seconds and increases toward 8 m/s over approximately 8 seconds. A downdraft of up to 3 m/s is also considered.

## 🔄 PID Control Architecture

![PID Control Architecture](images/pid_control_architecture.svg)

The control system contains three main loops:

1. **Position PID:** calculates the desired pitch angle from horizontal position error.
2. **Attitude PID:** controls pitch response to track the desired angle.
3. **Altitude PID:** adjusts thrust to maintain the desired altitude.

The wind disturbance enters the drone dynamics, while measured position, altitude, and attitude are fed back to their respective controllers.

## 🧪 Simulation Studies

Three studies are included in the project:

### Study 1: PID vs PD Controller

Compare the response of a PID controller with a PD-only controller under persistent wind.

### Study 2: Wind Intensity Analysis

Increase wind intensity and observe position error, required tilt angle, and tracking performance.

### Study 3: Position Gain Comparison

Vary position-loop gains to understand the trade-off between drift reduction and overshoot.

## 📊 Preliminary Results

The following values were reported by the preliminary Python equation-based study for the 8 m/s wind case. They have not yet been independently verified by running the Simulink model.

| Controller | Peak Position Error | Final Position Error | Peak Tilt |
|---|---:|---:|---:|
| PID | 0.49 m | 0 m | 9.6° |
| PD only | 0.70 m | 0.70 m | 9.5° |

### Observations

- Integral action helps compensate for persistent disturbances.
- PD-only control can retain a steady position offset.
- Stronger wind requires greater control effort and drone tilt.
- Aggressive position gains can increase overshoot.
- Altitude control performance depends on controller gains, thrust limits, and downdraft strength.

**Note:** These results are preliminary and must be reproduced in MATLAB/Simulink before being presented as verified simulation results.

## ▶️ How to Run the Project

### Requirements

- MATLAB
- Simulink

### Execution

Open MATLAB and navigate to the project directory.

```matlab
cd windshear_drone_project
build_drone_windshear_model
run_drone_windshear_study
```

All primary model parameters are configured in:

```matlab
drone_windshear_params.m
```

If a block-parameter error occurs, check the PID block parameter names supported by your installed MATLAB release.

## 📁 Project Structure

```text
windshear_drone_project/
│
├── README.md
├── build_drone_windshear_model.m
├── run_drone_windshear_study.m
├── drone_windshear_params.m
│
└── images/
    ├── quadrotor_windshear_model.svg
    └── pid_control_architecture.svg
```

Add MATLAB-generated plots to the `images` folder after running the simulation.

For example:

```markdown
![Position Response](images/position_response.png)

![Altitude Response](images/altitude_response.png)

![Wind Profile](images/wind_profile.png)
```

## 🚀 Future Improvements

- Implement full six-degree-of-freedom quadrotor dynamics.
- Add IMU and GPS sensor noise.
- Implement an Extended Kalman Filter for state estimation.
- Add realistic turbulence and gust models.
- Investigate disturbance-observer-based compensation.
- Compare PID with LQR and Model Predictive Control.
- Perform controller gain optimization and robustness testing.
- Validate the model against experimental flight data.

## 🎓 Applications

- UAV flight control research
- Drone stability analysis
- Wind disturbance rejection
- PID controller design
- MATLAB/Simulink-based avionics projects
- Academic research in autonomous aerial vehicles

## ⚠️ Disclaimer

This is an educational simulation project. The model is simplified and is not flight-certified. The controller gains and reported results should not be used for real-world flight without appropriate modelling, testing, and validation.

## 👨‍💻 Author

**Project:** Quadrotor Drone Wind Shear Simulation Using PID Control

**Domain:** Avionics Engineering | UAV Flight Control | Control Systems

**Tools:** MATLAB, Simulink, Python

