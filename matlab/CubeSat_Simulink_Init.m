%% CubeSat_Simulink_Init.m — Workspace Initialization for Simulink Model
%  Run this script BEFORE opening/simulating CubeSat_Simulink.slx
%  Loads all parameters into the MATLAB workspace.
%
%  CubeDynamics Project — MATLAB/Simulink Module
%  Authors: ARYA M G C, Ashwin R, Nithivalavan N, Jayaraj M, Vishal Meyyappan R
% =========================================================================

clear; clc;
fprintf('========================================\n');
fprintf('  CubeDynamics — Simulink Init\n');
fprintf('  Loading workspace parameters...\n');
fprintf('========================================\n\n');

%% ===================== SIMULATION PARAMETERS =====================
Ts        = 1/60;       % Sample time (s) — 60 Hz
sim_time  = 30;         % Total simulation duration (s)

%% ===================== CUBESAT PROPERTIES =====================
%  1U CubeSat: 10x10x10 cm, ~1 kg uniform cube
Ixx = 0.00167;   % kg·m²
Iyy = 0.00167;   % kg·m²
Izz = 0.00167;   % kg·m²
I_body = diag([Ixx, Iyy, Izz]);   % Full inertia tensor
I_vec  = [Ixx, Iyy, Izz];         % Vector form for element-wise ops

%% ===================== INITIAL CONDITIONS =====================
q0     = [1, 0, 0, 0];              % Identity quaternion
omega0 = [0.05, -0.03, 0.02];       % Initial tumble (rad/s)

%% ===================== PID CONTROLLER =====================
Kp = 0.01;
Ki = 0.001;
Kd = 0.005;

%% ===================== REACTION WHEEL LIMITS =====================
max_torque = 0.001;     % 1 mN·m per axis

%% ===================== SETPOINT =====================
setpoint_euler = [0, 0, 0];   % Target attitude [roll, pitch, yaw] (rad)

%% ===================== SENSOR NOISE =====================
sensor_noise_level = 0.01;    % 1% Gaussian noise
measurement_noise  = 0.01;    % EKF measurement noise R

%% ===================== EKF INITIAL STATE =====================
ekf_q0   = [1, 0, 0, 0];
ekf_w0   = [0, 0, 0];
ekf_P0   = 1.0;               % High initial covariance

%% ===================== DISTURBANCE =====================
disturbance_magnitude = 0.0001;   % N·m per axis

%% ===================== ANTI-WINDUP =====================
max_integral = 0.1;

%% ===================== PACKET LOSS (for telemetry model) =====================
packet_loss_rate = 0.02;   % 2% LoRa link drop rate

%% ===================== SUMMARY =====================
fprintf('  Parameters loaded into workspace:\n');
fprintf('    Ts             = %.4f s (%.0f Hz)\n', Ts, 1/Ts);
fprintf('    sim_time       = %.1f s\n', sim_time);
fprintf('    Inertia        = [%.5f, %.5f, %.5f] kg·m²\n', I_vec);
fprintf('    Initial omega  = [%.3f, %.3f, %.3f] rad/s\n', omega0);
fprintf('    PID            = Kp=%.4f  Ki=%.4f  Kd=%.4f\n', Kp, Ki, Kd);
fprintf('    Max torque     = %.1f mN·m\n', max_torque*1e3);
fprintf('    Sensor noise   = %.1f%%\n', sensor_noise_level*100);
fprintf('    Setpoint       = [%.1f, %.1f, %.1f] rad\n', setpoint_euler);
fprintf('\n  ✓ Ready. Open CubeSat_Simulink.slx and run.\n');
fprintf('========================================\n');
