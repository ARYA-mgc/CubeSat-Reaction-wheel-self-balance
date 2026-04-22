function [sensor_data, ekf_state_out] = FlightComputer(physics_state, ekf_state_in, pid_state_in, params)
% FLIGHTCOMPUTER  Simulates the ESP32 On-Board Computer (OBC)
%   Implements:
%     1. Sensor noise injection (Gaussian IMU noise)
%     2. Extended Kalman Filter (EKF) for state estimation
%     3. PID controller with anti-windup and torque saturation
%
%   Inputs:
%       physics_state  - struct with fields:
%                          .quaternion  [w, x, y, z]
%                          .omega       [wx, wy, wz]
%                          .time        scalar
%       ekf_state_in   - struct with fields:
%                          .quaternion  [w, x, y, z]  EKF estimate
%                          .omega       [wx, wy, wz]  EKF estimate
%                          .covariance  scalar         EKF covariance
%                          .converged   logical
%       pid_state_in   - struct with fields:
%                          .integral_error  [ex, ey, ez]
%                          .previous_error  [ex, ey, ez]
%       params         - struct with fields:
%                          .sensor_noise_level  (default 0.01)
%                          .Kp, .Ki, .Kd        PID gains
%                          .setpoint            [roll, pitch, yaw] rad
%                          .max_torque          (default 0.001 N·m)
%                          .max_integral        (default 0.1)
%                          .measurement_noise   (default 0.01)
%
%   Outputs:
%       sensor_data    - struct with fields:
%                          .quaternion       EKF-filtered quaternion
%                          .omega            EKF-filtered angular velocity
%                          .control_torque   [tx, ty, tz] PID output (N·m)
%                          .setpoint         [roll, pitch, yaw] target
%                          .ekf_covariance   scalar
%                          .ekf_converged    logical
%       ekf_state_out  - updated EKF state (for next iteration)
%
%   Reference: Dan Simon, "Optimal State Estimation"
%
%   CubeDynamics Project — MATLAB/Simulink Module
%   Authors: ARYA M G C, Ashwin R, Nithivalavan N, Jayaraj M, Vishal Meyyappan R
% -------------------------------------------------------------------------

    %% ================ SENSOR NOISE INJECTION ================
    % Add Gaussian noise to simulate real IMU measurements
    noise_q = randn(1, 4) * params.sensor_noise_level;
    noisy_quat = physics_state.quaternion + noise_q;
    noisy_quat = noisy_quat / norm(noisy_quat);    % Re-normalize

    noise_w = randn(1, 3) * params.sensor_noise_level;
    noisy_omega = physics_state.omega + noise_w;

    %% ================ EKF UPDATE ================
    % Simplified scalar-covariance Kalman filter
    % Matches the TypeScript FlightComputer.processSensorData()

    P = ekf_state_in.covariance;
    R = params.measurement_noise;

    % Kalman gain
    K = P / (P + R);

    % Update quaternion estimate (weighted average)
    ekf_quat = ekf_state_in.quaternion + K * (noisy_quat - ekf_state_in.quaternion);
    ekf_quat = ekf_quat / norm(ekf_quat);  % Normalize

    % Update angular velocity estimate
    ekf_omega = ekf_state_in.omega + K * (noisy_omega - ekf_state_in.omega);

    % Update covariance (reduce uncertainty)
    P_new = (1 - K) * P + 0.001;

    % Check convergence
    converged = P_new < 0.1;

    % Pack EKF output state
    ekf_state_out.quaternion = ekf_quat;
    ekf_state_out.omega      = ekf_omega;
    ekf_state_out.covariance = P_new;
    ekf_state_out.converged  = converged;

    %% ================ PID CONTROLLER ================
    % Convert EKF quaternion estimate to Euler angles
    [roll, pitch, yaw] = QuatToEuler(ekf_quat);
    current_euler = [roll, pitch, yaw];

    % Compute error: setpoint - current
    error = params.setpoint - current_euler;

    % Wrap angles to [-pi, pi]
    error = wrapToPi_custom(error);

    % Integral term with anti-windup
    integral_error = pid_state_in.integral_error + error;
    integral_error = max(-params.max_integral, min(params.max_integral, integral_error));

    % Derivative term
    derivative = error - pid_state_in.previous_error;

    % PID output
    torque = params.Kp * error + params.Ki * integral_error + params.Kd * derivative;

    % Saturate torque (reaction wheel limits)
    torque = max(-params.max_torque, min(params.max_torque, torque));

    %% ================ PACK OUTPUT ================
    sensor_data.quaternion      = ekf_quat;
    sensor_data.omega           = ekf_omega;
    sensor_data.control_torque  = torque;
    sensor_data.setpoint        = params.setpoint;
    sensor_data.ekf_covariance  = P_new;
    sensor_data.ekf_converged   = converged;

    % Update PID state (caller must persist these)
    sensor_data.pid_integral_error  = integral_error;
    sensor_data.pid_previous_error  = error;

end

%% ================ LOCAL HELPER ================
function wrapped = wrapToPi_custom(angles)
% Wrap angles to [-pi, pi] range
    wrapped = mod(angles + pi, 2*pi) - pi;
end
