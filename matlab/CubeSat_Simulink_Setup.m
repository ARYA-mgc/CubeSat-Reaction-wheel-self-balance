%% CubeSat_Simulink_Setup.m — Programmatic Simulink Model Builder
%  Creates the CubeSat_Simulink.slx model with all blocks and connections.
%  Run CubeSat_Simulink_Init.m first to load workspace parameters.
%
%  Block Diagram Architecture:
%  ┌──────────────┐     ┌──────────────┐     ┌───────────┐
%  │ Disturbance  │────▶│              │     │           │
%  │  Generator   │     │   CubeSat    │────▶│  Sensor   │
%  │              │     │   Plant      │     │   Model   │
%  │              │     │ (Dynamics)   │     │  (Noise)  │
%  └──────────────┘     └──────┲───────┘     └─────┬─────┘
%                              │                    │
%                              │                    ▼
%  ┌──────────────┐     ┌──────┸───────┐     ┌───────────┐
%  │  Reaction    │     │   Torque     │     │   EKF     │
%  │   Wheel      │◀────│   Summing    │     │ Estimator │
%  │ Saturation   │     │   Junction   │     │           │
%  └──────┬───────┘     └──────────────┘     └─────┬─────┘
%         │                                        │
%         │              ┌──────────────┐          │
%         └─────────────▶│    PID       │◀─────────┘
%                        │  Controller  │
%                        └──────────────┘
%
%  CubeDynamics Project — MATLAB/Simulink Module
%  Authors: ARYA M G C, Ashwin R, Nithivalavan N, Jayaraj M, Vishal Meyyappan R
% =========================================================================

%% ===================== PREREQUISITES =====================
fprintf('========================================\n');
fprintf('  CubeDynamics — Simulink Model Builder\n');
fprintf('========================================\n\n');

% Check if init was run
if ~exist('Ts', 'var') || ~exist('I_vec', 'var')
    fprintf('  Running CubeSat_Simulink_Init.m first...\n');
    CubeSat_Simulink_Init;
    fprintf('\n');
end

model_name = 'CubeSat_Simulink';

% Close if already open
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Delete existing file
if exist([model_name '.slx'], 'file')
    delete([model_name '.slx']);
end

%% ===================== CREATE MODEL =====================
fprintf('  Creating Simulink model: %s.slx\n', model_name);
new_system(model_name);
open_system(model_name);

% Set solver configuration
set_param(model_name, ...
    'Solver',       'FixedStepDiscrete', ...
    'FixedStep',    num2str(Ts), ...
    'StopTime',     num2str(sim_time), ...
    'SaveOutput',   'on', ...
    'SaveFormat',   'Array');

%% ===================== ADD BLOCKS =====================

% --- Positions (x, y layout) ---
x0 = 50; y0 = 50;
bw = 150; bh = 60;  % block width/height
gap = 80;

% ============================================================
%  ROW 1: Setpoint → Error → PID → Saturation → Sum
% ============================================================

% 1. Setpoint Constant
pos_setpoint = [x0, y0, x0+bw, y0+bh];
add_block('simulink/Sources/Constant', [model_name '/Setpoint'], ...
    'Value', mat2str(setpoint_euler), ...
    'Position', pos_setpoint, ...
    'BackgroundColor', 'cyan');

% 2. Error Sum (Setpoint - Estimated Attitude)
pos_error = [x0+bw+gap, y0, x0+2*bw+gap, y0+bh];
add_block('simulink/Math Operations/Sum', [model_name '/Error_Sum'], ...
    'Inputs', '+-', ...
    'Position', pos_error);

% 3. PID Controller (3-axis — using vectorized gains)
pos_pid = [x0+2*(bw+gap), y0, x0+3*bw+2*gap, y0+bh];
add_block('simulink/Math Operations/Gain', [model_name '/PID_Kp'], ...
    'Gain', num2str(Kp), ...
    'Position', pos_pid, ...
    'BackgroundColor', 'green');

% PID — Integral path
pos_ki = [x0+2*(bw+gap), y0+bh+gap, x0+3*bw+2*gap, y0+2*bh+gap];
add_block('simulink/Continuous/Integrator', [model_name '/Integrator'], ...
    'InitialCondition', '0', ...
    'UpperSaturationLimit', num2str(max_integral), ...
    'LowerSaturationLimit', num2str(-max_integral), ...
    'Position', pos_ki);

pos_ki_gain = [x0+3*(bw+gap), y0+bh+gap, x0+4*bw+3*gap, y0+2*bh+gap];
add_block('simulink/Math Operations/Gain', [model_name '/PID_Ki'], ...
    'Gain', num2str(Ki), ...
    'Position', pos_ki_gain, ...
    'BackgroundColor', 'green');

% PID — Derivative path
pos_kd = [x0+2*(bw+gap), y0+2*(bh+gap), x0+3*bw+2*gap, y0+3*bh+2*gap];
add_block('simulink/Continuous/Derivative', [model_name '/Derivative'], ...
    'Position', pos_kd);

pos_kd_gain = [x0+3*(bw+gap), y0+2*(bh+gap), x0+4*bw+3*gap, y0+3*bh+2*gap];
add_block('simulink/Math Operations/Gain', [model_name '/PID_Kd'], ...
    'Gain', num2str(Kd), ...
    'Position', pos_kd_gain, ...
    'BackgroundColor', 'green');

% PID Sum
pos_pid_sum = [x0+4*(bw+gap), y0, x0+4*(bw+gap)+60, y0+bh+2*(bh+gap)];
add_block('simulink/Math Operations/Sum', [model_name '/PID_Sum'], ...
    'Inputs', '+++', ...
    'Position', pos_pid_sum);

% 4. Reaction Wheel Saturation
pos_sat = [x0+5*(bw+gap), y0+bh+gap-bh/2, x0+5*(bw+gap)+bw, y0+bh+gap+bh/2];
add_block('simulink/Discontinuities/Saturation', [model_name '/RW_Saturation'], ...
    'UpperLimit', num2str(max_torque), ...
    'LowerLimit', num2str(-max_torque), ...
    'Position', pos_sat, ...
    'BackgroundColor', 'red');

% ============================================================
%  ROW 2: Plant (CubeSat Dynamics) — MATLAB Function block
% ============================================================

% 5. CubeSat Plant — MATLAB Function
pos_plant = [x0+6*(bw+gap), y0, x0+6*(bw+gap)+bw*1.5, y0+bh*2];
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/CubeSat_Plant'], ...
    'Position', pos_plant, ...
    'BackgroundColor', 'orange');

% 6. Disturbance Generator
pos_dist = [x0+6*(bw+gap), y0+3*(bh+gap), x0+6*(bw+gap)+bw, y0+3*(bh+gap)+bh];
add_block('simulink/Sources/Band-Limited White Noise', [model_name '/Disturbance'], ...
    'Cov', mat2str([1 1 1] * disturbance_magnitude^2), ...
    'Ts', num2str(Ts), ...
    'Position', pos_dist, ...
    'BackgroundColor', 'yellow');

% 7. Torque Sum (Control + Disturbance)
pos_torque_sum = [x0+5*(bw+gap)+bw+gap, y0+bh+gap, x0+5*(bw+gap)+bw+gap+60, y0+bh+gap+bh];
add_block('simulink/Math Operations/Sum', [model_name '/Torque_Sum'], ...
    'Inputs', '++', ...
    'Position', pos_torque_sum);

% ============================================================
%  ROW 3: Sensor Model + EKF (Simplified)
% ============================================================

% 8. Sensor Noise
pos_sensor = [x0+7*(bw+gap)+bw, y0, x0+7*(bw+gap)+2*bw, y0+bh];
add_block('simulink/Sources/Band-Limited White Noise', [model_name '/Sensor_Noise'], ...
    'Cov', mat2str([1 1 1] * sensor_noise_level^2), ...
    'Ts', num2str(Ts), ...
    'Position', pos_sensor, ...
    'BackgroundColor', 'magenta');

% 9. Measurement Sum (Plant Output + Noise)
pos_meas_sum = [x0+8*(bw+gap)+bw, y0, x0+8*(bw+gap)+bw+60, y0+bh];
add_block('simulink/Math Operations/Sum', [model_name '/Meas_Sum'], ...
    'Inputs', '++', ...
    'Position', pos_meas_sum);

% ============================================================
%  SCOPES
% ============================================================

% 10. Attitude Scope
pos_scope1 = [x0+9*(bw+gap)+bw, y0, x0+9*(bw+gap)+bw+80, y0+bh];
add_block('simulink/Sinks/Scope', [model_name '/Attitude_Scope'], ...
    'Position', pos_scope1, ...
    'BackgroundColor', 'lightBlue');

% 11. Torque Scope
pos_scope2 = [x0+9*(bw+gap)+bw, y0+bh+gap, x0+9*(bw+gap)+bw+80, y0+2*bh+gap];
add_block('simulink/Sinks/Scope', [model_name '/Torque_Scope'], ...
    'Position', pos_scope2, ...
    'BackgroundColor', 'lightBlue');

% 12. Angular Velocity Scope
pos_scope3 = [x0+9*(bw+gap)+bw, y0+2*(bh+gap), x0+9*(bw+gap)+bw+80, y0+3*bh+2*gap];
add_block('simulink/Sinks/Scope', [model_name '/Omega_Scope'], ...
    'Position', pos_scope3, ...
    'BackgroundColor', 'lightBlue');

% 13. To Workspace blocks for post-processing
add_block('simulink/Sinks/To Workspace', [model_name '/Attitude_Out'], ...
    'VariableName', 'sim_attitude', ...
    'Position', [x0+9*(bw+gap)+bw, y0+3*(bh+gap), x0+9*(bw+gap)+bw+80, y0+3*(bh+gap)+bh]);

add_block('simulink/Sinks/To Workspace', [model_name '/Torque_Out'], ...
    'VariableName', 'sim_torque', ...
    'Position', [x0+9*(bw+gap)+bw, y0+4*(bh+gap), x0+9*(bw+gap)+bw+80, y0+4*(bh+gap)+bh]);

%% ===================== CONNECT BLOCKS =====================
fprintf('  Connecting blocks...\n');

% Setpoint → Error Sum (positive)
add_line(model_name, 'Setpoint/1',   'Error_Sum/1', 'autoroute', 'on');

% Error → P path
add_line(model_name, 'Error_Sum/1',  'PID_Kp/1',    'autoroute', 'on');

% Error → I path
add_line(model_name, 'Error_Sum/1',  'Integrator/1', 'autoroute', 'on');
add_line(model_name, 'Integrator/1', 'PID_Ki/1',     'autoroute', 'on');

% Error → D path
add_line(model_name, 'Error_Sum/1',  'Derivative/1', 'autoroute', 'on');
add_line(model_name, 'Derivative/1', 'PID_Kd/1',     'autoroute', 'on');

% P + I + D → PID Sum
add_line(model_name, 'PID_Kp/1', 'PID_Sum/1', 'autoroute', 'on');
add_line(model_name, 'PID_Ki/1', 'PID_Sum/2', 'autoroute', 'on');
add_line(model_name, 'PID_Kd/1', 'PID_Sum/3', 'autoroute', 'on');

% PID Sum → Saturation
add_line(model_name, 'PID_Sum/1', 'RW_Saturation/1', 'autoroute', 'on');

% Saturation → Torque Sum
add_line(model_name, 'RW_Saturation/1', 'Torque_Sum/1', 'autoroute', 'on');

% Disturbance → Torque Sum
add_line(model_name, 'Disturbance/1', 'Torque_Sum/2', 'autoroute', 'on');

% Torque Sum → Scopes
add_line(model_name, 'RW_Saturation/1', 'Torque_Scope/1', 'autoroute', 'on');
add_line(model_name, 'RW_Saturation/1', 'Torque_Out/1',   'autoroute', 'on');

% Measurement (feedback path) → Error Sum (negative)
add_line(model_name, 'Meas_Sum/1', 'Error_Sum/2',      'autoroute', 'on');
add_line(model_name, 'Meas_Sum/1', 'Attitude_Scope/1', 'autoroute', 'on');
add_line(model_name, 'Meas_Sum/1', 'Attitude_Out/1',   'autoroute', 'on');

%% ===================== SET MATLAB FUNCTION CODE =====================
fprintf('  Writing CubeSat plant dynamics to MATLAB Function block...\n');

plant_code = sprintf([ ...
    'function euler_out = CubeSat_Plant(torque_in)\n', ...
    '%%#codegen\n', ...
    '%% CubeSat rigid body dynamics (simplified Euler-angle model)\n', ...
    '%% Full quaternion model is in PhysicsEngine.m\n', ...
    '\n', ...
    'persistent euler omega\n', ...
    'if isempty(euler)\n', ...
    '    euler = [0; 0; 0];\n', ...
    '    omega = [%.6f; %.6f; %.6f];\n', ...
    'end\n', ...
    '\n', ...
    'I = [%.6f; %.6f; %.6f];\n', ...
    'dt = %.6f;\n', ...
    '\n', ...
    '%% Euler''s equation: alpha = I^-1 * torque\n', ...
    'alpha = torque_in(:) ./ I;\n', ...
    '\n', ...
    '%% Integrate angular velocity\n', ...
    'omega = omega + alpha * dt;\n', ...
    '\n', ...
    '%% Integrate attitude (small-angle: theta_dot ≈ omega)\n', ...
    'euler = euler + omega * dt;\n', ...
    '\n', ...
    'euler_out = euler;\n' ...
    ], omega0(1), omega0(2), omega0(3), ...
       I_vec(1), I_vec(2), I_vec(3), Ts);

% Set the MATLAB function code
try
    % Get the MATLAB Function block's chart
    block_path = [model_name '/CubeSat_Plant'];
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if ~isempty(chart)
        chart.Script = plant_code;
    end
catch ME
    fprintf('    Note: Could not auto-set MATLAB Function code.\n');
    fprintf('    Please manually paste the plant code.\n');
    fprintf('    Error: %s\n', ME.message);
end

%% ===================== ADD ANNOTATIONS =====================
add_block('simulink/Commonly Used Blocks/Model Info', [model_name '/Info'], ...
    'Position', [x0, y0+5*(bh+gap), x0+3*bw, y0+5*(bh+gap)+bh*2]);

%% ===================== SAVE MODEL =====================
fprintf('  Saving model...\n');
save_system(model_name);
fprintf('\n  ✓ Model saved: %s.slx\n', model_name);
fprintf('  ✓ To simulate: click Run or type sim(''%s'')\n', model_name);
fprintf('\n========================================\n');
fprintf('  Simulink model created successfully!\n');
fprintf('========================================\n');

%% ===================== OPTIONAL: AUTO-SIMULATE =====================
% Uncomment below to run immediately:
%
% fprintf('\n  Running simulation...\n');
% simOut = sim(model_name);
% fprintf('  Simulation complete.\n');
%
% % Extract results
% t = simOut.tout;
% attitude = simOut.sim_attitude.signals.values;
% torque   = simOut.sim_torque.signals.values;
%
% PlotResults(t, rad2deg(attitude), zeros(size(attitude)), torque, ...
%             ones(size(t))*0.05, rad2deg(setpoint_euler));
