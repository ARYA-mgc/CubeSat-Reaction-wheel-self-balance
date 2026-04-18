# CubeDynamics

**Modular Hardware-in-the-Loop (HIL) CubeSat Simulation Platform**

A professional-grade web-based simulation platform for CubeSat attitude dynamics and control, built with React, TypeScript, and React Three Fiber.

## Features

- **Separated Architecture**: Physics Engine and Flight Computer (OBC) are completely decoupled
- **Real-time 3D Visualization**: Interactive CubeSat model with attitude representation
- **Advanced Control**: EKF state estimation and PID control implementation
- **Professional UI**: Dark "Mission Control" aesthetic with real-time telemetry charts
- **HIL Simulation**: Accurate sensor noise, packet loss, and disturbance injection

## Tech Stack

- **Framework**: React 18+ with TypeScript
- **Build Tool**: Vite
- **3D Graphics**: React Three Fiber & Drei
- **Charts**: Recharts
- **State Management**: Zustand
- **Styling**: Tailwind CSS
- **Icons**: Lucide React

## Getting Started

### Installation

```bash
npm install
```

### Development

```bash
npm run dev
```

Visit `http://localhost:3000` to see the simulation.

### Build

```bash
npm run build
```

## Architecture

### Core Components

1. **PhysicsEngine.ts**: Simulates rigid body dynamics using quaternions and Euler's equations
2. **FlightComputer.ts**: Implements EKF for state estimation and PID control
3. **SimulationLoop.ts**: Main simulation loop running at 60fps, bridges physics and control

### UI Components

- **SatVisualizer**: 3D CubeSat model with attitude visualization
- **TelemetryStream**: Real-time scrolling charts for attitude and torque
- **ControlPanel**: PID tuning and system configuration
- **StatusPanel**: Live telemetry readouts
- **LogConsole**: System event logging

## Features

### Physics Simulation

- Quaternion-based attitude representation (no gimbal lock)
- Rigid body dynamics integration
- Disturbance torque injection
- Reaction wheel control simulation

### Flight Computer

- Extended Kalman Filter (EKF) for state estimation
- PID controller with anti-windup
- Sensor noise simulation
- Control torque saturation

### User Interface

- Start/Stop/Reset controls
- Real-time fault injection
- PID gain tuning
- Target attitude setting
- LoRa link status monitoring
- Packet loss simulation

## License

MIT

## By

ARYA MGc (GitHub: ARYA-mgc)
