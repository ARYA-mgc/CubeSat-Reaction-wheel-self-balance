/**
 * FlightComputer.ts
 * Simulates the ESP32 On-Board Computer (OBC)
 * Implements EKF for state estimation and PID control
 */

import type { Vector3, Quaternion, PhysicsState } from './PhysicsEngine';
import { PhysicsEngine } from './PhysicsEngine';

export interface SensorData {
  quaternion: Quaternion;
  angularVelocity: Vector3;
  timestamp: number;
}

export interface PIDGains {
  Kp: number;
  Ki: number;
  Kd: number;
}

export interface ControlOutput {
  torque: Vector3;
  setpoint: Vector3; // Target attitude (Euler angles)
}

export class FlightComputer {
  private ekfState: {
    quaternion: Quaternion;
    angularVelocity: Vector3;
    covariance: number; // Simplified scalar covariance
  };
  
  private pidGains: PIDGains;
  private integralError: Vector3;
  private previousError: Vector3;
  private setpoint: Vector3; // Target Euler angles [roll, pitch, yaw]
  
  private sensorNoiseLevel: number;
  private ekfConverged: boolean;
  
  constructor() {
    // Initialize EKF state
    this.ekfState = {
      quaternion: { w: 1, x: 0, y: 0, z: 0 },
      angularVelocity: { x: 0, y: 0, z: 0 },
      covariance: 1.0, // High initial uncertainty
    };
    
    // PID gains (tuned for 1U CubeSat)
    this.pidGains = {
      Kp: 0.01,
      Ki: 0.001,
      Kd: 0.005,
    };
    
    this.integralError = { x: 0, y: 0, z: 0 };
    this.previousError = { x: 0, y: 0, z: 0 };
    this.setpoint = { x: 0, y: 0, z: 0 }; // Level attitude
    
    this.sensorNoiseLevel = 0.01; // 1% noise
    this.ekfConverged = false;
  }
  
  /**
   * Process noisy sensor data through EKF
   */
  processSensorData(perfectData: PhysicsState): SensorData {
    // Add sensor noise (simulate IMU noise)
    const noisyQuaternion = {
      w: perfectData.quaternion.w + this.gaussianNoise() * this.sensorNoiseLevel,
      x: perfectData.quaternion.x + this.gaussianNoise() * this.sensorNoiseLevel,
      y: perfectData.quaternion.y + this.gaussianNoise() * this.sensorNoiseLevel,
      z: perfectData.quaternion.z + this.gaussianNoise() * this.sensorNoiseLevel,
    };
    
    // Normalize noisy quaternion
    const norm = Math.sqrt(
      noisyQuaternion.w ** 2 +
      noisyQuaternion.x ** 2 +
      noisyQuaternion.y ** 2 +
      noisyQuaternion.z ** 2
    );
    
    noisyQuaternion.w /= norm;
    noisyQuaternion.x /= norm;
    noisyQuaternion.y /= norm;
    noisyQuaternion.z /= norm;
    
    const noisyAngularVelocity = {
      x: perfectData.angularVelocity.x + this.gaussianNoise() * this.sensorNoiseLevel,
      y: perfectData.angularVelocity.y + this.gaussianNoise() * this.sensorNoiseLevel,
      z: perfectData.angularVelocity.z + this.gaussianNoise() * this.sensorNoiseLevel,
    };
    
    // Simplified EKF update (Kalman gain calculation)
    const measurementNoise = 0.01;
    const kalmanGain = this.ekfState.covariance / (this.ekfState.covariance + measurementNoise);
    
    // Update estimate (weighted average of prediction and measurement)
    this.ekfState.quaternion = {
      w: this.ekfState.quaternion.w + kalmanGain * (noisyQuaternion.w - this.ekfState.quaternion.w),
      x: this.ekfState.quaternion.x + kalmanGain * (noisyQuaternion.x - this.ekfState.quaternion.x),
      y: this.ekfState.quaternion.y + kalmanGain * (noisyQuaternion.y - this.ekfState.quaternion.y),
      z: this.ekfState.quaternion.z + kalmanGain * (noisyQuaternion.z - this.ekfState.quaternion.z),
    };
    
    this.ekfState.angularVelocity = {
      x: this.ekfState.angularVelocity.x + kalmanGain * (noisyAngularVelocity.x - this.ekfState.angularVelocity.x),
      y: this.ekfState.angularVelocity.y + kalmanGain * (noisyAngularVelocity.y - this.ekfState.angularVelocity.y),
      z: this.ekfState.angularVelocity.z + kalmanGain * (noisyAngularVelocity.z - this.ekfState.angularVelocity.z),
    };
    
    // Update covariance (reduce uncertainty)
    this.ekfState.covariance = (1 - kalmanGain) * this.ekfState.covariance + 0.001;
    
    // Check convergence
    if (this.ekfState.covariance < 0.1) {
      this.ekfConverged = true;
    }
    
    return {
      quaternion: { ...this.ekfState.quaternion },
      angularVelocity: { ...this.ekfState.angularVelocity },
      timestamp: perfectData.time,
    };
  }
  
  /**
   * Compute PID control torque
   */
  computeControl(sensorData: SensorData): ControlOutput {
    // Convert current attitude to Euler angles
    const currentEuler = PhysicsEngine.quaternionToEuler(sensorData.quaternion);
    
    // Compute error (setpoint - current)
    const error = {
      x: this.setpoint.x - currentEuler[0],
      y: this.setpoint.y - currentEuler[1],
      z: this.setpoint.z - currentEuler[2],
    };
    
    // Wrap angles to [-π, π]
    error.x = this.wrapAngle(error.x);
    error.y = this.wrapAngle(error.y);
    error.z = this.wrapAngle(error.z);
    
    // Integral term (with anti-windup)
    const maxIntegral = 0.1;
    this.integralError.x = this.clamp(this.integralError.x + error.x, -maxIntegral, maxIntegral);
    this.integralError.y = this.clamp(this.integralError.y + error.y, -maxIntegral, maxIntegral);
    this.integralError.z = this.clamp(this.integralError.z + error.z, -maxIntegral, maxIntegral);
    
    // Derivative term
    const derivative = {
      x: error.x - this.previousError.x,
      y: error.y - this.previousError.y,
      z: error.z - this.previousError.z,
    };
    
    // PID output
    const torque = {
      x: this.pidGains.Kp * error.x + 
         this.pidGains.Ki * this.integralError.x + 
         this.pidGains.Kd * derivative.x,
      y: this.pidGains.Kp * error.y + 
         this.pidGains.Ki * this.integralError.y + 
         this.pidGains.Kd * derivative.y,
      z: this.pidGains.Kp * error.z + 
         this.pidGains.Ki * this.integralError.z + 
         this.pidGains.Kd * derivative.z,
    };
    
    // Saturate torque (reaction wheel limits)
    const maxTorque = 0.001; // 1 mN·m
    torque.x = this.clamp(torque.x, -maxTorque, maxTorque);
    torque.y = this.clamp(torque.y, -maxTorque, maxTorque);
    torque.z = this.clamp(torque.z, -maxTorque, maxTorque);
    
    // Save error for next iteration
    this.previousError = { ...error };
    
    return {
      torque,
      setpoint: { ...this.setpoint },
    };
  }
  
  /**
   * Update PID gains
   */
  setPIDGains(gains: Partial<PIDGains>): void {
    this.pidGains = { ...this.pidGains, ...gains };
  }
  
  /**
   * Update setpoint (target attitude)
   */
  setSetpoint(setpoint: Partial<Vector3>): void {
    this.setpoint = { ...this.setpoint, ...setpoint };
  }
  
  /**
   * Update sensor noise level
   */
  setSensorNoise(level: number): void {
    this.sensorNoiseLevel = level;
  }
  
  /**
   * Get EKF convergence status
   */
  isEKFConverged(): boolean {
    return this.ekfConverged;
  }
  
  /**
   * Reset controller
   */
  reset(): void {
    this.ekfState = {
      quaternion: { w: 1, x: 0, y: 0, z: 0 },
      angularVelocity: { x: 0, y: 0, z: 0 },
      covariance: 1.0,
    };
    this.integralError = { x: 0, y: 0, z: 0 };
    this.previousError = { x: 0, y: 0, z: 0 };
    this.ekfConverged = false;
  }
  
  /**
   * Generate Gaussian noise (Box-Muller transform)
   */
  private gaussianNoise(): number {
    const u1 = Math.random();
    const u2 = Math.random();
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  }
  
  /**
   * Wrap angle to [-π, π]
   */
  private wrapAngle(angle: number): number {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
  }
  
  /**
   * Clamp value
   */
  private clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, value));
  }
}
