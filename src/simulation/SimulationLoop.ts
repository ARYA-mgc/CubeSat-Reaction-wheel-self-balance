/**
 * SimulationLoop.ts
 * Main simulation loop that bridges Physics and Flight Computer
 * Runs at 60fps and syncs with Zustand store
 */

import { PhysicsEngine, type Vector3, type PhysicsState } from './PhysicsEngine';
import { FlightComputer, type SensorData, type ControlOutput } from './FlightComputer';

export interface SimulationData {
  physicsState: PhysicsState;
  sensorData: SensorData;
  controlOutput: ControlOutput;
  eulerAngles: [number, number, number];
  disturbance: Vector3;
  packetDropped: boolean;
}

export interface SimulationConfig {
  disturbanceMagnitude: number;
  packetLossRate: number;
  autoControl: boolean;
}

export class SimulationLoop {
  private physics: PhysicsEngine;
  private obc: FlightComputer;
  private animationId: number | null = null;
  private isRunning: boolean = false;
  
  private config: SimulationConfig;
  private dataHistory: SimulationData[] = [];
  private maxHistoryLength: number = 600; // 10 seconds at 60fps
  
  private onUpdate: ((data: SimulationData) => void) | null = null;
  private onLog: ((message: string) => void) | null = null;
  
  constructor(config: Partial<SimulationConfig> = {}) {
    this.physics = new PhysicsEngine();
    this.obc = new FlightComputer();
    
    this.config = {
      disturbanceMagnitude: 0.0001,
      packetLossRate: 0.02, // 2% packet loss
      autoControl: true,
      ...config,
    };
  }
  
  /**
   * Start simulation loop
   */
  start(): void {
    if (this.isRunning) return;
    
    this.isRunning = true;
    this.log('Simulation Started');
    this.loop();
  }
  
  /**
   * Stop simulation loop
   */
  stop(): void {
    if (!this.isRunning) return;
    
    this.isRunning = false;
    if (this.animationId !== null) {
      cancelAnimationFrame(this.animationId);
      this.animationId = null;
    }
    this.log('Simulation Stopped');
  }
  
  /**
   * Reset simulation
   */
  reset(): void {
    const wasRunning = this.isRunning;
    this.stop();
    
    this.physics.reset();
    this.obc.reset();
    this.dataHistory = [];
    
    this.log('Simulation Reset');
    
    if (wasRunning) {
      this.start();
    }
  }
  
  /**
   * Main simulation loop
   */
  private loop = (): void => {
    if (!this.isRunning) return;
    
    // Generate disturbance torque
    const disturbance = this.config.autoControl
      ? this.physics.injectDisturbance(this.config.disturbanceMagnitude)
      : { x: 0, y: 0, z: 0 };
    
    // Get perfect physics state
    const physicsState = this.physics.getState();
    
    // OBC processes noisy sensor data
    const sensorData = this.obc.processSensorData(physicsState);
    
    // OBC computes control torque
    const controlOutput = this.config.autoControl
      ? this.obc.computeControl(sensorData)
      : { torque: { x: 0, y: 0, z: 0 }, setpoint: { x: 0, y: 0, z: 0 } };
    
    // Physics engine steps forward
    this.physics.step(controlOutput.torque, disturbance);
    
    // Convert to Euler angles for display
    const eulerAngles = PhysicsEngine.quaternionToEuler(physicsState.quaternion);
    
    // Simulate packet loss
    const packetDropped = Math.random() < this.config.packetLossRate;
    
    // Create data snapshot
    const data: SimulationData = {
      physicsState,
      sensorData,
      controlOutput,
      eulerAngles,
      disturbance,
      packetDropped,
    };
    
    // Add to history
    this.dataHistory.push(data);
    if (this.dataHistory.length > this.maxHistoryLength) {
      this.dataHistory.shift();
    }
    
    // Notify observers (unless packet dropped)
    if (!packetDropped && this.onUpdate) {
      this.onUpdate(data);
    }
    
    // Log events periodically
    if (Math.random() < 0.01) { // ~1% chance per frame
      if (this.obc.isEKFConverged() && !packetDropped) {
        this.log(`EKF Converged - Roll: ${(eulerAngles[0] * 180 / Math.PI).toFixed(2)}°`);
      }
      if (packetDropped) {
        this.log('Packet Dropped - LoRa Link Interrupted');
      }
    }
    
    // Schedule next frame
    this.animationId = requestAnimationFrame(this.loop);
  };
  
  /**
   * Inject fault (large disturbance)
   */
  injectFault(): void {
    const faultTorque = {
      x: (Math.random() - 0.5) * 0.01,
      y: (Math.random() - 0.5) * 0.01,
      z: (Math.random() - 0.5) * 0.01,
    };
    
    this.physics.step(faultTorque, { x: 0, y: 0, z: 0 });
    this.log('⚠️ FAULT INJECTED - Large disturbance applied');
  }
  
  /**
   * Set initial angular velocity
   */
  setInitialConditions(angularVelocity: Vector3): void {
    this.physics.setInitialConditions(angularVelocity);
    this.log(`Initial ω set: [${angularVelocity.x.toFixed(3)}, ${angularVelocity.y.toFixed(3)}, ${angularVelocity.z.toFixed(3)}] rad/s`);
  }
  
  /**
   * Update configuration
   */
  updateConfig(config: Partial<SimulationConfig>): void {
    this.config = { ...this.config, ...config };
  }
  
  /**
   * Update OBC PID gains
   */
  setPIDGains(Kp?: number, Ki?: number, Kd?: number): void {
    this.obc.setPIDGains({ Kp, Ki, Kd });
    this.log(`PID Gains Updated - Kp: ${Kp?.toFixed(4)}, Ki: ${Ki?.toFixed(4)}, Kd: ${Kd?.toFixed(4)}`);
  }
  
  /**
   * Update OBC setpoint
   */
  setSetpoint(roll?: number, pitch?: number, yaw?: number): void {
    this.obc.setSetpoint({ 
      x: roll !== undefined ? roll * Math.PI / 180 : undefined, 
      y: pitch !== undefined ? pitch * Math.PI / 180 : undefined, 
      z: yaw !== undefined ? yaw * Math.PI / 180 : undefined 
    });
    this.log(`Setpoint Updated - Roll: ${roll}°, Pitch: ${pitch}°, Yaw: ${yaw}°`);
  }
  
  /**
   * Update sensor noise
   */
  setSensorNoise(level: number): void {
    this.obc.setSensorNoise(level);
    this.log(`Sensor Noise: ${(level * 100).toFixed(1)}%`);
  }
  
  /**
   * Get data history
   */
  getHistory(): SimulationData[] {
    return [...this.dataHistory];
  }
  
  /**
   * Get current data
   */
  getCurrentData(): SimulationData | null {
    return this.dataHistory.length > 0
      ? this.dataHistory[this.dataHistory.length - 1]
      : null;
  }
  
  /**
   * Register update callback
   */
  setUpdateCallback(callback: (data: SimulationData) => void): void {
    this.onUpdate = callback;
  }
  
  /**
   * Register log callback
   */
  setLogCallback(callback: (message: string) => void): void {
    this.onLog = callback;
  }
  
  /**
   * Internal logging
   */
  private log(message: string): void {
    if (this.onLog) {
      this.onLog(message);
    }
  }
  
  /**
   * Check if running
   */
  getIsRunning(): boolean {
    return this.isRunning;
  }
}
