function [roll, pitch, yaw] = QuatToEuler(q)
% QUATTOEULER  Convert quaternion to Euler angles (ZYX convention)
%   Matches the TypeScript PhysicsEngine.quaternionToEuler() implementation.
%
%   Input:
%       q  - quaternion [w, x, y, z]  (1x4)
%
%   Outputs:
%       roll   - rotation about X-axis (rad)
%       pitch  - rotation about Y-axis (rad)
%       yaw    - rotation about Z-axis (rad)
%
%   CubeDynamics Project — MATLAB/Simulink Module
% -------------------------------------------------------------------------

    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);

    % Roll (X-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz);
    cosr_cosp = 1 - 2 * (qx^2 + qy^2);
    roll = atan2(sinr_cosp, cosr_cosp);

    % Pitch (Y-axis rotation) — clamp to avoid NaN at gimbal lock
    sinp = 2 * (qw * qy - qz * qx);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi / 2;   % Gimbal lock
    else
        pitch = asin(sinp);
    end

    % Yaw (Z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy);
    cosy_cosp = 1 - 2 * (qy^2 + qz^2);
    yaw = atan2(siny_cosp, cosy_cosp);

end
