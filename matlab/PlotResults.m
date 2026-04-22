function PlotResults(time_vec, euler_history, omega_history, torque_history, covariance_history, setpoint)
% PLOTRESULTS  Generate publication-quality simulation result plots
%   Creates a 4-subplot figure showing attitude dynamics, angular velocity,
%   control torques, and EKF convergence.
%
%   Inputs:
%       time_vec           - (Nx1) time vector (s)
%       euler_history      - (Nx3) Euler angles [roll, pitch, yaw] (deg)
%       omega_history      - (Nx3) angular velocities [wx, wy, wz] (rad/s)
%       torque_history     - (Nx3) control torques [tx, ty, tz] (N·m)
%       covariance_history - (Nx1) EKF covariance over time
%       setpoint           - [roll, pitch, yaw] target attitude (deg)
%
%   CubeDynamics Project — MATLAB/Simulink Module
%   Authors: ARYA M G C, Ashwin R, Nithivalavan N, Jayaraj M, Vishal Meyyappan R
% -------------------------------------------------------------------------

    % Color palette (matching CubeDynamics web UI dark theme)
    color_roll  = [0.239, 0.584, 0.882];   % #3D95E1 — Blue
    color_pitch = [0.882, 0.337, 0.408];   % #E15668 — Red
    color_yaw   = [0.302, 0.800, 0.498];   % #4DCC7F — Green
    color_cov   = [0.949, 0.749, 0.286];   % #F2BF49 — Gold

    fig = figure('Name', 'CubeDynamics — Simulation Results', ...
                 'Color', [0.08 0.08 0.12], ...
                 'Position', [100, 100, 1200, 800]);

    %% ---- Subplot 1: Euler Angles (Attitude) ----
    ax1 = subplot(2, 2, 1);
    hold on; grid on;
    plot(time_vec, euler_history(:,1), 'Color', color_roll,  'LineWidth', 1.5, 'DisplayName', 'Roll');
    plot(time_vec, euler_history(:,2), 'Color', color_pitch, 'LineWidth', 1.5, 'DisplayName', 'Pitch');
    plot(time_vec, euler_history(:,3), 'Color', color_yaw,   'LineWidth', 1.5, 'DisplayName', 'Yaw');
    % Setpoint reference lines
    yline(setpoint(1), '--', 'Color', color_roll,  'LineWidth', 0.8, 'Alpha', 0.5);
    yline(setpoint(2), '--', 'Color', color_pitch, 'LineWidth', 0.8, 'Alpha', 0.5);
    yline(setpoint(3), '--', 'Color', color_yaw,   'LineWidth', 0.8, 'Alpha', 0.5);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Attitude (Euler Angles)');
    legend('Location', 'northeast');
    set(ax1, 'Color', [0.12 0.12 0.16], 'XColor', [0.7 0.7 0.7], ...
             'YColor', [0.7 0.7 0.7], 'GridColor', [0.3 0.3 0.3]);

    %% ---- Subplot 2: Angular Velocity ----
    ax2 = subplot(2, 2, 2);
    hold on; grid on;
    plot(time_vec, omega_history(:,1), 'Color', color_roll,  'LineWidth', 1.5, 'DisplayName', '\omega_x');
    plot(time_vec, omega_history(:,2), 'Color', color_pitch, 'LineWidth', 1.5, 'DisplayName', '\omega_y');
    plot(time_vec, omega_history(:,3), 'Color', color_yaw,   'LineWidth', 1.5, 'DisplayName', '\omega_z');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    title('Body-Frame Angular Velocity');
    legend('Location', 'northeast');
    set(ax2, 'Color', [0.12 0.12 0.16], 'XColor', [0.7 0.7 0.7], ...
             'YColor', [0.7 0.7 0.7], 'GridColor', [0.3 0.3 0.3]);

    %% ---- Subplot 3: Control Torques ----
    ax3 = subplot(2, 2, 3);
    hold on; grid on;
    plot(time_vec, torque_history(:,1)*1e3, 'Color', color_roll,  'LineWidth', 1.5, 'DisplayName', '\tau_x');
    plot(time_vec, torque_history(:,2)*1e3, 'Color', color_pitch, 'LineWidth', 1.5, 'DisplayName', '\tau_y');
    plot(time_vec, torque_history(:,3)*1e3, 'Color', color_yaw,   'LineWidth', 1.5, 'DisplayName', '\tau_z');
    yline( 1, 'r--', 'LineWidth', 0.8, 'DisplayName', 'Saturation');
    yline(-1, 'r--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    xlabel('Time (s)');
    ylabel('Torque (mN\cdotm)');
    title('Reaction Wheel Control Torques');
    legend('Location', 'northeast');
    set(ax3, 'Color', [0.12 0.12 0.16], 'XColor', [0.7 0.7 0.7], ...
             'YColor', [0.7 0.7 0.7], 'GridColor', [0.3 0.3 0.3]);

    %% ---- Subplot 4: EKF Covariance (Convergence) ----
    ax4 = subplot(2, 2, 4);
    hold on; grid on;
    plot(time_vec, covariance_history, 'Color', color_cov, 'LineWidth', 1.5, 'DisplayName', 'EKF Covariance');
    yline(0.1, 'g--', 'LineWidth', 1.0, 'DisplayName', 'Convergence Threshold');
    xlabel('Time (s)');
    ylabel('Covariance');
    title('EKF Convergence');
    legend('Location', 'northeast');
    set(ax4, 'Color', [0.12 0.12 0.16], 'XColor', [0.7 0.7 0.7], ...
             'YColor', [0.7 0.7 0.7], 'GridColor', [0.3 0.3 0.3]);

    %% ---- Overall Figure Title ----
    sgtitle('CubeDynamics — CubeSat Reaction Wheel Simulation', ...
            'Color', [0.9 0.9 0.9], 'FontSize', 14, 'FontWeight', 'bold');

    fprintf('  [PlotResults] Figure generated successfully.\n');

end
