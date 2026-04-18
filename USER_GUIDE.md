# CubeDynamics - User Guide

## Overview

CubeDynamics is a professional Hardware-in-the-Loop (HIL) simulation platform for CubeSat attitude dynamics and control. This guide will help you understand and use all features of the platform.

## Interface Layout

The platform uses a "Mission Control" dark theme layout with the following panels:

### 1. **Toolbar** (Top)
- **Start/Pause Button**: Begin or pause the simulation
- **Reset Button**: Reset the simulation to initial conditions
- **Inject Fault Button**: Apply a large random disturbance torque
- **LoRa Link Status**: Shows the telemetry connection status (Connected/Degraded/Disconnected)

### 2. **Control Panel** (Left)
Contains all system configuration controls:

#### PID Controller
- **Kp (Proportional)**: Immediate response to error (default: 0.01)
- **Ki (Integral)**: Eliminates steady-state error (default: 0.001)
- **Kd (Derivative)**: Dampens oscillations (default: 0.005)

#### Target Attitude
- **Roll, Pitch, Yaw**: Target attitude in degrees

#### System Parameters
- **Sensor Noise Level**: Simulates IMU noise (default: 1%)

#### Initial Conditions
- **ωx, ωy, ωz**: Initial angular velocity in rad/s

### 3. **3D Visualizer** (Center Top)
Real-time 3D visualization showing:
- **CubeSat Model**: 1U cube (10x10x10 cm) with solar panels
- **Body Axes**: RGB arrows (Red=X, Green=Y, Blue=Z)
- **Angular Velocity Vector**: Yellow arrow showing rotation axis and magnitude
- **Reference Grid**: Orbital frame reference
- **Stars Background**: Space environment visualization

### 4. **Status Panel** (Right)
Real-time telemetry readouts:
- **Attitude**: Roll, Pitch, Yaw in degrees
- **Angular Velocity**: ωx, ωy, ωz in rad/s
- **Control Torque**: τx, τy, τz from reaction wheels in mN·m
- **Simulation Time**: Current simulation time in seconds

### 5. **Telemetry Charts** (Center Bottom Left)
Two real-time scrolling charts:
- **Attitude Chart**: Shows Roll (red), Pitch (green), Yaw (cyan) over time
- **Torque Chart**: Shows control torques τx (red), τy (green), τz (cyan)

### 6. **Log Console** (Bottom Right)
System event logging showing:
- EKF convergence status
- Packet drops
- System parameter changes
- Fault injections

## How to Use

### Basic Operation

1. **Starting the Simulation**
   - Click the green "Start" button
   - The 3D model will begin moving based on physics
   - Charts will start plotting data
   - The log console will show events

2. **Observing Behavior**
   - Watch the 3D model rotate in space
   - Observe the controller trying to stabilize the attitude
   - Monitor the telemetry charts for convergence
   - Check the log for EKF status

3. **Pausing/Resetting**
   - Click "Pause" to freeze the simulation
   - Click "Reset" to return to initial conditions

### Advanced Features

#### 1. **PID Tuning**
To tune the controller for better performance:

- **Increase Kp**: Faster response, but may cause oscillations
- **Increase Ki**: Eliminates steady-state error, but may cause instability
- **Increase Kd**: Reduces oscillations, but may make system sluggish

Example aggressive tuning:
- Kp = 0.02
- Ki = 0.005
- Kd = 0.01

#### 2. **Target Attitude Control**
Set a desired attitude:
- Enter angles in degrees
- Click "Update Setpoint"
- Watch the controller maneuver to the new target

Example: Point CubeSat 45° in roll:
- Roll = 45°
- Pitch = 0°
- Yaw = 0°

#### 3. **Disturbance Testing**
Test controller robustness:
- Click "Inject Fault" while running
- Observe how quickly the system recovers
- Check the disturbance magnitude in the log

#### 4. **Initial Conditions**
Set initial rotation:
- Enter angular velocities (e.g., ωx = 0.1 rad/s)
- Click "Set Initial ω"
- Press "Reset" to apply
- Press "Start" to see the motion

#### 5. **Sensor Noise Testing**
Test EKF performance:
- Increase "Sensor Noise Level" (e.g., 0.05 = 5%)
- Click "Update Noise"
- Observe EKF filtering in the charts

## Understanding the Physics

### Attitude Representation
- **Quaternions**: Used internally to avoid gimbal lock
- **Euler Angles**: Displayed for human readability (Roll, Pitch, Yaw)

### Dynamics
The simulation implements Euler's equations:
```
τ = I·ω̇ + ω × (I·ω)
```
Where:
- τ = torque (control + disturbance)
- I = moment of inertia
- ω = angular velocity
- ω̇ = angular acceleration

### Control System
1. **Physics Engine** → Generates perfect state
2. **Sensor Model** → Adds noise to simulate IMU
3. **EKF** → Filters noisy measurements
4. **PID Controller** → Computes required torque
5. **Actuators** → Apply torque (with saturation)
6. **Physics Engine** → Updates state

## Packet Loss Simulation

The platform simulates LoRa telemetry with 2% packet loss:
- **Connected** (Green): No packets dropped recently
- **Degraded** (Amber): Occasional packet loss
- **Disconnected** (Red): Simulation stopped

When a packet is dropped:
- UI briefly shows no update
- Log shows "Packet Dropped" message
- Link status turns amber

## Tips for Best Results

1. **Start Simple**: Begin with default PID gains and zero initial conditions
2. **Tune Gradually**: Change one PID parameter at a time
3. **Watch Convergence**: Wait for EKF to converge before judging performance
4. **Use Log**: The log console provides valuable debugging information
5. **Test Robustness**: Use "Inject Fault" to verify controller stability
6. **Experiment**: Try different initial spins and target attitudes

## Troubleshooting

### CubeSat spinning uncontrollably
- Reduce Kp gain
- Increase Kd gain
- Check initial angular velocity isn't too high

### Controller not reaching setpoint
- Increase Kp gain
- Increase Ki gain
- Check for control torque saturation in charts

### Charts not updating
- Check if simulation is running (green "Start" should show "Pause")
- Check LoRa link status
- Try clicking Reset

### 3D view not rotating smoothly
- Check browser performance
- Close other applications
- Reduce sensor noise level

## Technical Specifications

- **Simulation Rate**: 60 Hz (fixed timestep)
- **CubeSat Mass**: 1 kg
- **CubeSat Size**: 1U (10x10x10 cm)
- **Inertia**: 0.00167 kg·m² (each axis)
- **Max Torque**: 1 mN·m (per axis)
- **Sensor Noise**: Gaussian, configurable
- **Packet Loss**: 2% (configurable in code)

## Keyboard Shortcuts

(Future feature - to be implemented)

## Export Data

(Future feature - to be implemented)

## Credits

Built with:
- React 18 + TypeScript
- React Three Fiber (3D graphics)
- Recharts (telemetry visualization)
- Zustand (state management)
- Tailwind CSS (styling)

---

**CubeDynamics** - Professional CubeSat HIL Simulation Platform
Version 1.0.0
