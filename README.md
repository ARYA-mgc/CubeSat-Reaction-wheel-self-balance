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

---

## Team & Contributors

This project was built collaboratively by the following team:

| Role | Name | GitHub |
|------|------|--------|
| 👑 Team Leader — System Architecture & Embedded HIL Integration | ARYA M G C | [@ARYA-mgc](https://github.com/ARYA-mgc) |
| ⚙️ Control Systems & Attitude Dynamics | Ashwin R | [@ashwinr-act-cit](https://github.com/ashwinr-act-cit) |
| 💻 Simulation Framework & Software Development | Nithivalavan N | [@Nithi-tech](https://github.com/Nithi-tech) |
| 🔧 Hardware Design & Power Systems | Jayaraj M | [@jayarajMd](https://github.com/jayarajMd) |
| 📡 Telemetry & Communication Interfaces | Vishal Meyyappan R | [@vishal-r07](https://github.com/vishal-r07) |

### Collaborators

- [@ARYA-mgc](https://github.com/ARYA-mgc) — System Architecture & Embedded HIL Integration
- [@ashwinr-act-cit](https://github.com/ashwinr-act-cit) — Control Systems & Attitude Dynamics
- [@Nithi-tech](https://github.com/Nithi-tech) — Simulation Framework & Software Development
- [@jayarajMd](https://github.com/jayarajMd) — Hardware Design & Power Systems
- [@vishal-r07](https://github.com/vishal-r07) — Telemetry & Communication Interfaces

---

## License

MIT License

Copyright (c) 2026 ARYA-mgc, ashwinr-act-cit, jayarajMd, Nithi-tech, vishal-r07

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
