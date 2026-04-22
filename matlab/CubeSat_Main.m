%% CubeSat_Main.m — CubeDynamics MATLAB Simulation
%  Main entry point: runs the full CubeSat attitude control simulation
%  matching the TypeScript web simulation (PhysicsEngine + FlightComputer).
%
%  Simulation Loop (discrete, fixed-step):
%     1. Generate environmental disturbance torque
%     2. Read perfect physics state
%     3. OBC processes noisy sensor data → EKF estimate
%     4. OBC computes PID control torque
%     5. Physics engine steps forward with control + disturbance
%     6. Store telemetry for plotting
%
%  CubeDynamics Project — MATLAB/Simulink Module
%  Authors: ARYA M G C, Ashwin R, Nithivalavan N, Jayaraj M, Vishal Meyyappan R
% =========================================================================

clear; clc; close all;
fprintf('========================================\n');
fprintf('  CubeDynamics — MATLAB Simulation\n');
fprintf('  CubeSat Reaction Wheel Self-Balance\n');
fprintf('========================================\n\n');

%% ===================== CONFIGURATION =====================

% --- Simulation Parameters ---
sim_time     = 30;           % Total simulation time (s)
dt           = 1/60;         % Time step matching 60 Hz loop
N_steps      = round(sim_time / dt);

% --- 1U CubeSat Physical Properties ---
%  10 cm x 10 cm x 10 cm, ~1 kg
inertia = [0.00167, 0.00167, 0.00167];   % [Ixx, Iyy, Izz] (kg·m²)

% --- PID Controller Gains ---
params.Kp = 0.01;
params.Ki = 0.001;
params.Kd = 0.005;

% --- Setpoint (target attitude) ---
%  Set to zero for level attitude; change to test maneuvers
params.setpoint = [0, 0, 0];   % [roll, pitch, yaw] in radians

% --- Sensor / EKF Parameters ---
params.sensor_noise_level = 0.01;    % 1% Gaussian noise
params.measurement_noise  = 0.01;    % EKF measurement noise
params.max_integral       = 0.1;     % Anti-windup clamp
params.max_torque         = 0.001;   % 1 mN·m reaction wheel limit

% --- Disturbance Parameters ---
disturbance_magnitude = 0.0001;      % Environmental torque magnitude (N·m)

% --- Initial Conditions ---
initial_omega = [0.05, -0.03, 0.02]; % Initial angular velocity (rad/s)
%  (Simulates a tumbling CubeSat after deployment)

fprintf('  Configuration:\n');
fprintf('    Sim time     : %.1f s  (%d steps at %.0f Hz)\n', sim_time, N_steps, 1/dt);
fprintf('    Inertia      : [%.5f, %.5f, %.5f] kg·m²\n', inertia);
fprintf('    PID          : Kp=%.4f  Ki=%.4f  Kd=%.4f\n', params.Kp, params.Ki, params.Kd);
fprintf('    Sensor noise : %.1f%%\n', params.sensor_noise_level * 100);
fprintf('    Max torque   : %.1f mN·m\n', params.max_torque * 1e3);
fprintf('    Initial ω    : [%.3f, %.3f, %.3f] rad/s\n\n', initial_omega);

%% ===================== INITIALIZE STATE =====================

% Physics state
state.quaternion = [1, 0, 0, 0];   % Identity quaternion
state.omega      = initial_omega;
state.time       = 0;

% EKF state
ekf_state.quaternion = [1, 0, 0, 0];
ekf_state.omega      = [0, 0, 0];
ekf_state.covariance = 1.0;         % High initial uncertainty
ekf_state.converged  = false;

% PID state
pid_state.integral_error = [0, 0, 0];
pid_state.previous_error = [0, 0, 0];

%% ===================== PREALLOCATE HISTORY =====================

time_vec           = zeros(N_steps, 1);
euler_history      = zeros(N_steps, 3);   % [roll, pitch, yaw] in degrees
omega_history      = zeros(N_steps, 3);
torque_history     = zeros(N_steps, 3);
covariance_history = zeros(N_steps, 1);
disturbance_hist   = zeros(N_steps, 3);

%% ===================== SIMULATION LOOP =====================

fprintf('  Running simulation...\n');
tic;

ekf_converge_step = NaN;

for k = 1:N_steps

    % 1. Generate random environmental disturbance torque
    disturbance = (rand(1,3) - 0.5) * disturbance_magnitude;

    % 2. OBC processes noisy sensor data through EKF + computes PID torque
    [sensor_data, ekf_state] = FlightComputer(state, ekf_state, pid_state, params);

    % Update PID state for next iteration
    pid_state.integral_error = sensor_data.pid_integral_error;
    pid_state.previous_error = sensor_data.pid_previous_error;

    control_torque = sensor_data.control_torque;

    % 3. Physics engine steps forward
    state = PhysicsEngine(state, control_torque, disturbance, inertia, dt);

    % 4. Convert quaternion to Euler angles for telemetry
    [roll, pitch, yaw] = QuatToEuler(state.quaternion);

    % 5. Store telemetry
    time_vec(k)           = state.time;
    euler_history(k,:)    = rad2deg([roll, pitch, yaw]);
    omega_history(k,:)    = state.omega;
    torque_history(k,:)   = control_torque;
    covariance_history(k) = sensor_data.ekf_covariance;
    disturbance_hist(k,:) = disturbance;

    % Log EKF convergence event (once)
    if sensor_data.ekf_converged && isnan(ekf_converge_step)
        ekf_converge_step = k;
        fprintf('    [t=%.2fs] EKF Converged (step %d)\n', state.time, k);
    end
end

elapsed = toc;
fprintf('  Simulation complete in %.3f s (%.1fx real-time)\n\n', elapsed, sim_time / elapsed);

%% ===================== PRINT SUMMARY =====================

fprintf('  Final State:\n');
fprintf('    Attitude : Roll=%.2f°  Pitch=%.2f°  Yaw=%.2f°\n', euler_history(end,:));
fprintf('    ω        : [%.4f, %.4f, %.4f] rad/s\n', omega_history(end,:));
fprintf('    Torque   : [%.6f, %.6f, %.6f] N·m\n', torque_history(end,:));
fprintf('    EKF Cov  : %.6f  (converged=%s)\n', covariance_history(end), ...
        mat2str(covariance_history(end) < 0.1));

% Check settling
final_error = abs(euler_history(end,:) - rad2deg(params.setpoint));
if all(final_error < 1.0)
    fprintf('    Status   : ✓ SETTLED (error < 1° on all axes)\n');
else
    fprintf('    Status   : ✗ NOT YET SETTLED (error = [%.2f, %.2f, %.2f]°)\n', final_error);
end

%% ===================== PLOT RESULTS =====================

fprintf('\n  Generating plots...\n');
PlotResults(time_vec, euler_history, omega_history, torque_history, ...
            covariance_history, rad2deg(params.setpoint));

fprintf('\n========================================\n');
fprintf('  Simulation finished. Plots displayed.\n');
fprintf('========================================\n');
