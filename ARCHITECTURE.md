# CubeDynamics - Architecture Documentation

## System Architecture

CubeDynamics implements a clean separation between the "Physics World" and "Flight Software" to accurately simulate Hardware-in-the-Loop (HIL) architecture.

```
┌─────────────────────────────────────────────────────┐
│                   React UI Layer                    │
│  (Toolbar, Charts, 3D Visualizer, Control Panel)   │
└─────────────────────┬───────────────────────────────┘
                      │
                      ↓
┌─────────────────────────────────────────────────────┐
│              Zustand State Store                    │
│         (Bridge between UI and Simulation)          │
└─────────────────────┬───────────────────────────────┘
                      │
                      ↓
┌─────────────────────────────────────────────────────┐
│              Simulation Loop (60 Hz)                │
│    • Orchestrates Physics & Flight Computer         │
│    • Handles packet loss simulation                 │
│    • Manages data history                           │
└──────────────┬─────────────────────┬────────────────┘
               │                     │
               ↓                     ↓
    ┌──────────────────┐  ┌──────────────────────┐
    │ Physics Engine   │  │  Flight Computer     │
    │  (Real World)    │  │      (OBC)           │
    │                  │  │                      │
    │ • Quaternions    │  │ • EKF Estimator      │
    │ • Euler Eqs      │  │ • PID Controller     │
    │ • Disturbances   │  │ • Sensor Noise       │
    └──────────────────┘  └──────────────────────┘
```

## Core Classes

### 1. PhysicsEngine (`src/simulation/PhysicsEngine.ts`)

**Purpose**: Simulates the "real world" rigid body dynamics of the CubeSat.

**Key Features**:
- **Quaternion State**: Uses quaternions for attitude to avoid gimbal lock
- **Euler Integration**: Integrates rigid body equations of motion
- **Disturbance Model**: Can inject random environmental torques
- **Pure Physics**: No knowledge of control system

**State Variables**:
```typescript
{
  quaternion: { w, x, y, z },     // Attitude quaternion
  angularVelocity: { x, y, z },   // Body-frame angular velocity (rad/s)
  time: number                     // Simulation time (s)
}
```

**Key Methods**:
- `step(controlTorque, disturbanceTorque)`: Integrate one timestep
- `injectDisturbance(magnitude)`: Generate random torque
- `reset()`: Return to initial state
- `setInitialConditions(angularVelocity)`: Set initial spin
- `quaternionToEuler(q)`: Convert for display

**Physics Equations**:

Euler's equations of motion:
```
ω̇ = I⁻¹ · (τ - ω × (I·ω))
```

Quaternion kinematics:
```
q̇ = 0.5 · q ⊗ [0, ωx, ωy, ωz]
```

### 2. FlightComputer (`src/simulation/FlightComputer.ts`)

**Purpose**: Simulates the ESP32 On-Board Computer (OBC) with estimation and control.

**Key Features**:
- **EKF State Estimator**: Filters noisy sensor data
- **PID Controller**: Computes control torques
- **Sensor Model**: Adds Gaussian noise to perfect measurements
- **No Physics Access**: Only receives sensor data (mimics real hardware)

**Components**:

#### Extended Kalman Filter (EKF)
- Estimates true attitude from noisy measurements
- Uses Kalman gain to weight prediction vs measurement
- Tracks covariance to monitor convergence

#### PID Controller
- **Proportional**: Immediate response to error
- **Integral**: Eliminates steady-state error (with anti-windup)
- **Derivative**: Dampens oscillations
- **Saturation**: Limits torque to actuator capabilities (1 mN·m)

**Key Methods**:
- `processSensorData(perfectData)`: Add noise, run EKF
- `computeControl(sensorData)`: Run PID to get torque
- `setPIDGains(Kp, Ki, Kd)`: Update controller
- `setSetpoint(roll, pitch, yaw)`: Update target
- `setSensorNoise(level)`: Adjust noise magnitude

**Control Law**:
```
τ = Kp·e + Ki·∫e·dt + Kd·(de/dt)
```
Where `e = setpoint - current attitude`

### 3. SimulationLoop (`src/simulation/SimulationLoop.ts`)

**Purpose**: Main loop that bridges physics and flight software.

**Key Responsibilities**:
1. **Orchestration**: Calls physics and OBC in correct order
2. **Data Flow**:
   ```
   Physics → Sensors → EKF → PID → Physics
   ```
3. **Telemetry**: Manages data history for plotting
4. **Packet Loss**: Simulates 2% LoRa packet drops
5. **Logging**: Generates system events

**Loop Sequence** (60 Hz):
```typescript
1. Generate disturbance torque
2. Get perfect physics state
3. OBC processes noisy sensor data → EKF estimate
4. OBC computes control torque → PID output
5. Physics steps forward with control + disturbance
6. Store data point in history
7. Notify UI (unless packet dropped)
8. Schedule next frame
```

**Key Methods**:
- `start()`: Begin animation loop
- `stop()`: Pause simulation
- `reset()`: Reinitialize everything
- `injectFault()`: Apply large disturbance
- `updateConfig(config)`: Change parameters

## State Management

### Zustand Store (`src/store/simulationStore.ts`)

**Purpose**: Central state management connecting simulation to UI.

**State**:
- `isRunning`: Simulation active flag
- `currentData`: Latest telemetry frame
- `logs`: System event history
- `linkStatus`: LoRa connection status
- `config`: Simulation parameters
- `pidGains`: Controller tuning
- `setpoint`: Target attitude

**Actions**:
All UI interactions go through store actions which call simulation methods.

## UI Components

### Component Tree
```
App
├── Toolbar
│   ├── Start/Pause/Reset buttons
│   └── Link status indicator
├── ControlPanel (left)
│   ├── PID gain inputs
│   ├── Setpoint inputs
│   ├── Noise level slider
│   └── Initial condition inputs
├── SatVisualizer (center-top)
│   ├── 3D CubeSat model
│   ├── Angular velocity vector
│   ├── Body axes
│   └── Reference grid
├── StatusPanel (right)
│   ├── Attitude readout
│   ├── Angular velocity readout
│   └── Torque readout
├── TelemetryStream (center-bottom-left)
│   ├── Attitude chart
│   └── Torque chart
└── LogConsole (bottom-right)
    └── Event log
```

### Data Flow

1. **Simulation → Store**:
   ```typescript
   simulation.setUpdateCallback((data) => {
     set({ currentData: data });
   });
   ```

2. **Store → UI**:
   ```typescript
   const currentData = useSimulationStore((state) => state.currentData);
   ```

3. **UI → Store → Simulation**:
   ```typescript
   const { setPIDGains } = useSimulationStore();
   setPIDGains(Kp, Ki, Kd); // Calls simulation.setPIDGains()
   ```

## Key Design Decisions

### 1. **Quaternions for Physics**
- Avoids gimbal lock singularities
- Easier to integrate numerically
- Converted to Euler angles only for UI display

### 2. **Separated Physics & OBC**
- Mimics real HIL architecture
- OBC only sees noisy sensor data
- Physics is "ground truth"

### 3. **60 Hz Fixed Timestep**
- Uses `requestAnimationFrame` for smooth rendering
- Fixed dt = 1/60 s for consistent physics
- Decoupled from UI render rate

### 4. **EKF State Estimation**
- Demonstrates sensor fusion concept
- Shows difference between noisy measurements and estimate
- Convergence visible in logs

### 5. **Packet Loss Simulation**
- 2% drop rate mimics LoRa telemetry
- UI gracefully handles missing frames
- Link status indicator shows quality

### 6. **Zustand for State**
- Simpler than Redux
- No boilerplate
- React hooks integration
- Easy to debug

## Performance Considerations

### Optimization Strategies

1. **React Three Fiber**:
   - Uses `useFrame` for 60 Hz updates
   - Only re-renders 3D scene when needed
   - `isAnimationActive={false}` on charts to prevent re-renders

2. **Chart Performance**:
   - Fixed 300-point buffer (5 seconds)
   - Only updates when new data arrives
   - Uses `ResponsiveContainer` for resize efficiency

3. **State Updates**:
   - Batched in Zustand
   - Only subscribed components re-render
   - Selector functions prevent unnecessary updates

4. **History Management**:
   - Rolling 600-frame buffer (10 seconds)
   - Automatically prunes old data
   - Prevents memory leaks

## Testing Strategy

### Unit Tests (Future)
- `PhysicsEngine`: Test quaternion math, integration accuracy
- `FlightComputer`: Test EKF convergence, PID response
- `SimulationLoop`: Test data flow, packet loss

### Integration Tests (Future)
- Full control loop convergence
- Disturbance rejection
- Setpoint tracking

### Manual Tests
1. **Stability**: Start with zero initial conditions, should converge
2. **Disturbance**: Inject fault, should recover
3. **Setpoint**: Change target, should maneuver smoothly
4. **PID Tuning**: Increase Kp → faster but oscillatory
5. **EKF**: Increase noise → slower convergence

## Future Enhancements

### Phase 2 Features
1. **Export Data**: CSV download of telemetry
2. **Multiple CubeSats**: Fleet simulation
3. **Orbit Propagation**: J2 perturbations
4. **Magnetic Torquers**: Alternative actuators
5. **Sun Sensor**: Additional sensor simulation
6. **Monte Carlo**: Batch runs with random parameters

### Phase 3 Features
1. **Real Hardware Integration**: Connect to actual ESP32
2. **WebSocket Telemetry**: Live data from hardware
3. **Mission Planning**: Upload attitude timeline
4. **Replay Mode**: Playback recorded data

## Development Setup

### File Structure
```
cubedynamics/
├── src/
│   ├── simulation/
│   │   ├── PhysicsEngine.ts      # Rigid body dynamics
│   │   ├── FlightComputer.ts     # EKF + PID
│   │   └── SimulationLoop.ts     # Main orchestrator
│   ├── store/
│   │   └── simulationStore.ts    # Zustand state
│   ├── components/
│   │   ├── Toolbar.tsx           # Top controls
│   │   ├── ControlPanel.tsx      # Left config panel
│   │   ├── SatVisualizer.tsx     # 3D viewer
│   │   ├── TelemetryStream.tsx   # Charts
│   │   ├── StatusPanel.tsx       # Right telemetry
│   │   └── LogConsole.tsx        # Event log
│   ├── utils/
│   │   └── cn.ts                 # Tailwind helper
│   ├── App.tsx                   # Main layout
│   └── main.tsx                  # Entry point
├── public/
│   └── vite.svg                  # Favicon
├── index.html                    # HTML shell
├── package.json                  # Dependencies
├── vite.config.ts                # Build config
├── tailwind.config.js            # Styles config
└── tsconfig.json                 # TypeScript config
```

### Adding New Features

1. **New Simulation Feature**:
   - Add to `PhysicsEngine` or `FlightComputer`
   - Expose through `SimulationLoop` method
   - Add action to `simulationStore`
   - Create UI control in relevant component

2. **New UI Component**:
   - Create in `src/components/`
   - Subscribe to store with `useSimulationStore`
   - Add to `App.tsx` layout
   - Style with Tailwind classes

3. **New Telemetry Chart**:
   - Add data collection in `TelemetryStream`
   - Create new `LineChart` instance
   - Use consistent color scheme
   - Ensure `isAnimationActive={false}`

## Debugging Tips

1. **Check Browser Console**: All errors appear here
2. **Use Log Console**: System events logged in UI
3. **Watch State**: Use React DevTools to inspect store
4. **Monitor Charts**: Visualize system behavior
5. **Reduce Complexity**: Start with zero initial conditions

## References

- **Attitude Dynamics**: Wertz, "Spacecraft Attitude Determination and Control"
- **Kalman Filtering**: Dan Simon, "Optimal State Estimation"
- **PID Control**: Åström & Hägglund, "PID Controllers"
- **Quaternion Math**: Kuipers, "Quaternions and Rotation Sequences"

---

**CubeDynamics** - Built for CubeSat research and education
