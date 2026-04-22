# MATLAB & Simulink — CubeDynamics

MATLAB/Simulink implementation of the CubeSat reaction wheel attitude control simulation,
matching the TypeScript web simulation exactly.

## Quick Start

### MATLAB Script (no Simulink required)

```matlab
cd matlab/
CubeSat_Main      % Runs full simulation and generates plots
```

### Simulink Model

```matlab
cd matlab/
CubeSat_Simulink_Init    % Load parameters into workspace
CubeSat_Simulink_Setup   % Build the Simulink model (.slx)
% Then click "Run" in Simulink, or:
sim('CubeSat_Simulink')
```

## File Descriptions

| File | Description |
|------|-------------|
| `CubeSat_Main.m` | **Main script** — 60 Hz simulation loop, plots results |
| `PhysicsEngine.m` | Rigid body dynamics (Euler's equations + quaternion kinematics) |
| `FlightComputer.m` | EKF state estimation + PID controller with anti-windup |
| `QuatToEuler.m` | Quaternion → Euler angle conversion (ZYX) |
| `QuatMultiply.m` | Hamilton quaternion product |
| `PlotResults.m` | Publication-quality 4-panel figure |
| `CubeSat_Simulink_Init.m` | Loads workspace parameters for Simulink |
| `CubeSat_Simulink_Setup.m` | Programmatically builds the Simulink `.slx` model |

## Parameters

All parameters match the TypeScript web simulation:

| Parameter | Value | Unit |
|-----------|-------|------|
| CubeSat mass | 1.0 | kg |
| CubeSat size | 10×10×10 | cm |
| Inertia (Ixx=Iyy=Izz) | 0.00167 | kg·m² |
| PID Kp | 0.01 | — |
| PID Ki | 0.001 | — |
| PID Kd | 0.005 | — |
| Max torque | 1.0 | mN·m |
| Sensor noise | 1% | — |
| Sim timestep | 1/60 | s |
| Disturbance | 0.1 | mN·m |

## Expected Output

Running `CubeSat_Main.m` produces a 4-panel figure:

1. **Attitude (Euler Angles)** — Roll, pitch, yaw converging to setpoint
2. **Angular Velocity** — Body-frame ω decaying to zero
3. **Control Torques** — Reaction wheel commands (with saturation limits)
4. **EKF Convergence** — Covariance dropping below threshold

## Requirements

- MATLAB R2020b or later
- Simulink (only for `.slx` model; MATLAB scripts work standalone)
- No additional toolboxes required

## Authors

- ARYA M G C — System Architecture & Embedded HIL Integration
- Ashwin R — Control Systems & Attitude Dynamics
- Nithivalavan N — Simulation Framework & Software Development
- Jayaraj M — Hardware Design & Power Systems
- Vishal Meyyappan R — Telemetry & Communication Interfaces
