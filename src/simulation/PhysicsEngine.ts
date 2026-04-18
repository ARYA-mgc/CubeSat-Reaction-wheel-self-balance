/**
 * PhysicsEngine.ts
 * Simulates the "Real World" - rigid body dynamics of the CubeSat
 * Uses quaternions for attitude representation to avoid gimbal lock
 */

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

export interface PhysicsState {
  quaternion: Quaternion;  // Attitude quaternion
  angularVelocity: Vector3; // Angular velocity in body frame (rad/s)
  time: number;             // Simulation time (s)
}

export class PhysicsEngine {
  private state: PhysicsState;
  private inertia: Vector3; // Moment of inertia for 1U CubeSat (kg·m²)
  private dt: number;       // Time step (s)
  
  constructor(dt: number = 1 / 60) {
    this.dt = dt;
    
    // 1U CubeSat inertia approximation (10x10x10 cm, 1kg)
    this.inertia = {
      x: 0.00167, // I_xx
      y: 0.00167, // I_yy
      z: 0.00167, // I_zz
    };
    
    // Initialize with identity quaternion and zero angular velocity
    this.state = {
      quaternion: { w: 1, x: 0, y: 0, z: 0 },
      angularVelocity: { x: 0, y: 0, z: 0 },
      time: 0,
    };
  }
  
  /**
   * Main physics step - integrates Euler's equations of motion
   * τ = I·ω̇ + ω × (I·ω)
   */
  step(controlTorque: Vector3, disturbanceTorque: Vector3): PhysicsState {
    const ω = this.state.angularVelocity;
    const I = this.inertia;
    
    // Total torque = control + disturbance
    const τ = {
      x: controlTorque.x + disturbanceTorque.x,
      y: controlTorque.y + disturbanceTorque.y,
      z: controlTorque.z + disturbanceTorque.z,
    };
    
    // Euler's equation: ω̇ = I⁻¹·(τ - ω × (I·ω))
    const Iω = {
      x: I.x * ω.x,
      y: I.y * ω.y,
      z: I.z * ω.z,
    };
    
    // Cross product: ω × (I·ω)
    const cross = {
      x: ω.y * Iω.z - ω.z * Iω.y,
      y: ω.z * Iω.x - ω.x * Iω.z,
      z: ω.x * Iω.y - ω.y * Iω.x,
    };
    
    // Angular acceleration
    const α = {
      x: (τ.x - cross.x) / I.x,
      y: (τ.y - cross.y) / I.y,
      z: (τ.z - cross.z) / I.z,
    };
    
    // Integrate angular velocity (simple Euler integration)
    this.state.angularVelocity = {
      x: ω.x + α.x * this.dt,
      y: ω.y + α.y * this.dt,
      z: ω.z + α.z * this.dt,
    };
    
    // Update quaternion using angular velocity
    this.updateQuaternion();
    
    // Increment time
    this.state.time += this.dt;
    
    return { ...this.state };
  }
  
  /**
   * Updates quaternion from angular velocity
   * q̇ = 0.5 * q ⊗ [0, ωx, ωy, ωz]
   */
  private updateQuaternion(): void {
    const q = this.state.quaternion;
    const ω = this.state.angularVelocity;
    
    // Quaternion derivative
    const qDot = {
      w: 0.5 * (-q.x * ω.x - q.y * ω.y - q.z * ω.z),
      x: 0.5 * (q.w * ω.x + q.y * ω.z - q.z * ω.y),
      y: 0.5 * (q.w * ω.y + q.z * ω.x - q.x * ω.z),
      z: 0.5 * (q.w * ω.z + q.x * ω.y - q.y * ω.x),
    };
    
    // Integrate quaternion
    q.w += qDot.w * this.dt;
    q.x += qDot.x * this.dt;
    q.y += qDot.y * this.dt;
    q.z += qDot.z * this.dt;
    
    // Normalize quaternion to prevent drift
    const norm = Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
  }
  
  /**
   * Inject random disturbance torque (simulates environmental effects)
   */
  injectDisturbance(magnitude: number = 0.0001): Vector3 {
    return {
      x: (Math.random() - 0.5) * magnitude,
      y: (Math.random() - 0.5) * magnitude,
      z: (Math.random() - 0.5) * magnitude,
    };
  }
  
  /**
   * Get current state
   */
  getState(): PhysicsState {
    return { ...this.state };
  }
  
  /**
   * Reset simulation
   */
  reset(): void {
    this.state = {
      quaternion: { w: 1, x: 0, y: 0, z: 0 },
      angularVelocity: { x: 0, y: 0, z: 0 },
      time: 0,
    };
  }
  
  /**
   * Set initial angular velocity (for testing)
   */
  setInitialConditions(angularVelocity: Vector3): void {
    this.state.angularVelocity = { ...angularVelocity };
  }
  
  /**
   * Convert quaternion to Euler angles (for visualization)
   * Returns [roll, pitch, yaw] in radians
   */
  static quaternionToEuler(q: Quaternion): [number, number, number] {
    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    const pitch = Math.abs(sinp) >= 1
      ? Math.sign(sinp) * Math.PI / 2
      : Math.asin(sinp);
    
    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);
    
    return [roll, pitch, yaw];
  }
}
