function [state_out] = PhysicsEngine(state_in, control_torque, disturbance_torque, inertia, dt)
% PHYSICSENGINE  Simulates rigid body dynamics of the CubeSat
%   Uses quaternion attitude representation to avoid gimbal lock.
%   Integrates Euler's equations of motion:
%       omega_dot = I^{-1} * (tau - omega x (I * omega))
%   Quaternion kinematics:
%       q_dot = 0.5 * q (x) [0, omega_x, omega_y, omega_z]
%
%   Inputs:
%       state_in            - struct with fields:
%                               .quaternion  [w, x, y, z]  (1x4)
%                               .omega       [wx, wy, wz]  (1x3)  body-frame angular velocity (rad/s)
%                               .time        scalar         simulation time (s)
%       control_torque      - [tx, ty, tz]  (1x3) control torque from reaction wheels (N·m)
%       disturbance_torque  - [tx, ty, tz]  (1x3) environmental disturbance torque (N·m)
%       inertia             - [Ixx, Iyy, Izz] (1x3) principal moments of inertia (kg·m²)
%       dt                  - scalar, time step (s)
%
%   Outputs:
%       state_out           - updated state struct (same fields as state_in)
%
%   Reference: Wertz, "Spacecraft Attitude Determination and Control"
%
%   CubeDynamics Project — MATLAB/Simulink Module
%   Authors: ARYA M G C, Ashwin R, Nithivalavan N, Jayaraj M, Vishal Meyyappan R
% -------------------------------------------------------------------------

    % Unpack state
    q = state_in.quaternion;   % [w, x, y, z]
    omega = state_in.omega;    % [wx, wy, wz]

    % Total torque = control + disturbance
    tau = control_torque + disturbance_torque;

    % ------------------------------------------------------------------
    % Euler's equation:  omega_dot = I^{-1} * (tau - omega x (I * omega))
    % ------------------------------------------------------------------
    I_omega = inertia .* omega;   % Element-wise: [Ixx*wx, Iyy*wy, Izz*wz]

    % Cross product: omega x (I * omega)
    cross_term = cross(omega, I_omega);

    % Angular acceleration
    alpha = (tau - cross_term) ./ inertia;

    % Integrate angular velocity (Euler integration)
    omega_new = omega + alpha * dt;

    % ------------------------------------------------------------------
    % Quaternion kinematics:  q_dot = 0.5 * q (x) [0, omega]
    % ------------------------------------------------------------------
    wx = omega_new(1);
    wy = omega_new(2);
    wz = omega_new(3);

    % Quaternion derivative
    q_dot = 0.5 * [ -q(2)*wx - q(3)*wy - q(4)*wz;   % dw/dt
                      q(1)*wx + q(3)*wz - q(4)*wy;   % dx/dt
                      q(1)*wy + q(4)*wx - q(2)*wz;   % dy/dt
                      q(1)*wz + q(2)*wy - q(3)*wx ];  % dz/dt

    % Integrate quaternion (Euler integration)
    q_new = q(:) + q_dot * dt;

    % Normalize quaternion to prevent drift
    q_new = q_new / norm(q_new);

    % ------------------------------------------------------------------
    % Pack output state
    % ------------------------------------------------------------------
    state_out.quaternion = q_new(:)';    % Ensure row vector [w, x, y, z]
    state_out.omega      = omega_new;
    state_out.time       = state_in.time + dt;

end
