%% drone_windshear_params.m
% Parameters for "Effect of wind shear on a PID-controlled quadrotor drone".
% Run through build_drone_windshear_model (it evaluates this in the base workspace).
%
% Planar (x-z) quadrotor: horizontal position x, altitude z, pitch theta.
% Values are representative of a ~1.5 kg quadcopter; replace with your drone's data.

%% --- Vehicle -------------------------------------------------------------
m  = 1.5;        % [kg]       mass
g  = 9.81;       % [m/s^2]
J  = 0.03;       % [kg m^2]   pitch inertia
cdx = 0.30;      % [N s/m]    horizontal linear drag coefficient
cdz = 0.40;      % [N s/m]    vertical linear drag coefficient
kq  = 0.02;      % [N m s]    rotational damping
tau_m  = 0.03;   % [s]        motor/ESC lag (thrust and torque)
taumax = 1.5;    % [N m]      max pitch torque
theta_max = deg2rad(25);   % [rad] max commanded tilt

%% --- Wind shear (gust front with downdraft and height gradient) ---------
ShearOn = 1;     % 1 = wind active, 0 = calm air
Ug  = 8;         % [m/s] final horizontal wind speed after the shear event
Aw  = 3;         % [m/s] peak downdraft
t0  = 10;        % [s]   shear onset
Tshear = 8;      % [s]   duration of the wind transition
z0  = 30;        % [m]   hover altitude above ground (for the wind height profile)
alphas = 0.2;    % [-]   power-law exponent: U(z) = U * ((z0+z)/z0)^alphas

%% --- PID gains: (Kp, Ki, Kd, filter N) -------------------------------------
% Altitude loop: z error -> thrust delta about hover (limited to +/- m*g)
z_Kp = 13.5;  z_Ki = 5.0;   z_Kd = 7.2;    z_N = 20;
% Position loop: x error -> pitch command (limited to +/- theta_max)
x_Kp = 0.23;  x_Ki = 0.03;  x_Kd = 0.245;  x_N = 10;
% Attitude loop: pitch error -> pitch torque (PD)
th_Kp = 6.75; th_Ki = 0.0;  th_Kd = 0.63;  th_N = 40;

%% --- Commands and simulation -------------------------------------------------
xcmd = 0;        % [m] horizontal position command
zcmd = 0;        % [m] altitude command (deviation from hover altitude)
tstop = 50;      % [s]
