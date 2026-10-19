# Effect of Wind Shear on a PID-Controlled Quadrotor Drone (Simulink)
M.Tech avionics mini-project: position and altitude control of a drone under wind shear.

## 1. Problem statement
A quadrotor hovers at 30 m and is controlled by cascaded PID loops. At t = 10 s a gust front
arrives: the horizontal wind rises smoothly to Ug (default 8 m/s) over 8 s, with a downdraft
(peak 3 m/s) and a height-dependent wind profile (power law). Quantify how the shear degrades
position and altitude hold, and how integral action, loop gains and actuator limits shape the response.

## 2. Files
| File | Purpose |
|---|---|
| `drone_windshear_params.m` | All parameters (vehicle, wind, PID gains). Edit here. |
| `build_drone_windshear_model.m` | Builds `drone_windshear_pid.slx` from basic blocks |
| `run_drone_windshear_study.m` | Studies A/B/C, figures, `drone_windshear_results.csv` |
| `reference_python/` | Independent Python implementation used to tune gains and check results |
| `reference_drone_shear_response.png` | Reference plot of the expected behaviour |

## 3. How to run
```matlab
cd windshear_drone_project
build_drone_windshear_model
run_drone_windshear_study
```
Needs MATLAB + Simulink only. The Simulink model is generated programmatically. The Python
reference was run and verified, but MATLAB was not available where I built this, so if the builder
reports a block-parameter error on your release, adjust that line (PID block parameter names are the
most likely to differ between releases).

## 4. Model (planar x–z quadrotor)
- m·ẍ = T·sinθ − cdx·(ẋ − u_w)
- m·z̈ = T·cosθ − m·g − cdz·(ż − w_w)
- J·θ̈ = τ − kq·θ̇
- Motors: first-order lag (30 ms) on thrust T and torque τ.
- Wind: u_w = Ug·(1−cos πs)/2 · ((z0+z)/z0)^α, w_w = −Aw·sin²(πs), s = clip((t−t0)/T, 0, 1).

How shear hurts the drone:
1. Relative-airspeed drag pushes the drone sideways, so the controller must tilt to hold position.
   At 8 m/s the steady tilt is about 9.6°.
2. Tilting reduces vertical thrust (T·cosθ), coupling the horizontal disturbance into altitude.
3. The downdraft adds a vertical drag disturbance.
4. Tilt and thrust limits can saturate in strong shear, causing windup.

## 5. Controller
- Altitude: `PID_altitude`, thrust delta limited to ±m·g (Kp 13.5, Ki 5, Kd 7.2, N 20).
- Position: `PID_position` gives the pitch command, limited to ±25° (Kp 0.23, Ki 0.03, Kd 0.245, N 10).
- Attitude: `PID_attitude` gives the pitch torque (Kp 6.75, Kd 0.63, N 40).
All PIDs use a filtered derivative and clamping anti-windup. The attitude loop is tuned for roughly
15 rad/s, the position loop for roughly 1.5 rad/s, and the altitude loop for roughly 3 rad/s
(the usual bandwidth separation).

## 6. Results (from the verified reference simulation, Ug = 8 m/s)
| Case | Peak x error | Final x error | Peak z error | Peak pitch | Peak thrust |
|---|---|---|---|---|---|
| PID | 0.49 m | 0.00 m | 0.06 m | 9.6° | 16.0 N |
| PD only (Ki = 0) | 0.70 m | **0.70 m (never recovers)** | 0.09 m | 9.5° | 16.0 N |

Intensity sweep (peak position error, PID / PD): Ug = 4 → 0.25 / 0.35 m, 12 → 0.73 / 1.04 m,
20 → 1.18 / 1.68 m. Peak tilt reaches 23° at 20 m/s, close to the 25° limit.

Position-loop gain sets (Ug = 8):
| Kp, Ki, Kd | Peak x error | Step overshoot (calm) | Wind recovery time |
|---|---|---|---|
| 0.10, 0.01, 0.15 | 1.23 m | 10% | 36 s |
| 0.23, 0.03, 0.245 | 0.49 m | 8% | 24 s |
| 0.50, 0.08, 0.40 | 0.21 m | 7% | 16 s |
| 1.0, 0.2, 0.6 | 0.09 m | 54% | 11 s |

Findings to discuss:
1. Wind acts as a persistent force, so a PD controller leaves a steady-state offset; integral action is
   needed to remove it.
2. Higher position gains cut drift and recovery time, but beyond a point overshoot explodes because
   the attitude loop and motor lag erode phase margin (last row).
3. The tilt needed to hold position grows linearly with wind speed, so strong shear drives the drone
   toward the tilt limit; thrust demand rises as T = m·g/cosθ.
4. Altitude error is small because the altitude loop is fast, but it comes from the cosθ coupling and
   the downdraft, and it grows with shear strength.

## 7. Extensions for the report
- Extend to full 6-DOF (roll, pitch, yaw, 3-D position) with mixer and per-motor thrust limits.
- Add Dryden turbulence or a measured wind profile; add sensor noise and an estimator.
- Add wind feedforward from an estimated disturbance (disturbance observer) and compare to PID.
- Optimize gains (`pidtune`, ITAE with `fmincon`) and plot margins versus tilt angle.
- Add battery sag and motor saturation to study loss of authority in strong shear.
