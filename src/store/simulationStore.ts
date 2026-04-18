/**
 * simulationStore.ts
 * Zustand store for managing simulation state across components
 */

import { create } from 'zustand';
import { SimulationLoop, type SimulationData, type SimulationConfig } from '../simulation/SimulationLoop';

interface LogEntry {
  timestamp: Date;
  message: string;
}

interface SimulationStore {
  // Simulation instance
  simulation: SimulationLoop;
  
  // State
  isRunning: boolean;
  currentData: SimulationData | null;
  logs: LogEntry[];
  linkStatus: 'connected' | 'disconnected' | 'degraded';
  
  // Configuration
  config: SimulationConfig;
  pidGains: { Kp: number; Ki: number; Kd: number };
  setpoint: { roll: number; pitch: number; yaw: number };
  
  // Actions
  start: () => void;
  stop: () => void;
  reset: () => void;
  injectFault: () => void;
  updateConfig: (config: Partial<SimulationConfig>) => void;
  setPIDGains: (Kp: number, Ki: number, Kd: number) => void;
  setSetpoint: (roll: number, pitch: number, yaw: number) => void;
  setSensorNoise: (level: number) => void;
  setInitialConditions: (wx: number, wy: number, wz: number) => void;
  addLog: (message: string) => void;
}

export const useSimulationStore = create<SimulationStore>()((set, get) => {
  // Create simulation instance
  const simulation = new SimulationLoop({
    disturbanceMagnitude: 0.0001,
    packetLossRate: 0.02,
    autoControl: true,
  });
  
  // Set up callbacks
  simulation.setUpdateCallback((data: SimulationData) => {
    set({ 
      currentData: data,
      linkStatus: data.packetDropped ? 'degraded' : 'connected',
    });
  });
  
  simulation.setLogCallback((message: string) => {
    get().addLog(message);
  });
  
  return {
    simulation,
    isRunning: false,
    currentData: null,
    logs: [],
    linkStatus: 'disconnected',
    
    config: {
      disturbanceMagnitude: 0.0001,
      packetLossRate: 0.02,
      autoControl: true,
    },
    
    pidGains: {
      Kp: 0.01,
      Ki: 0.001,
      Kd: 0.005,
    },
    
    setpoint: {
      roll: 0,
      pitch: 0,
      yaw: 0,
    },
    
    start: () => {
      const { simulation } = get();
      simulation.start();
      set({ isRunning: true, linkStatus: 'connected' });
    },
    
    stop: () => {
      const { simulation } = get();
      simulation.stop();
      set({ isRunning: false, linkStatus: 'disconnected' });
    },
    
    reset: () => {
      const { simulation } = get();
      simulation.reset();
      set({ 
        currentData: null,
        logs: [],
        linkStatus: 'disconnected',
      });
    },
    
    injectFault: () => {
      const { simulation } = get();
      simulation.injectFault();
    },
    
    updateConfig: (config: Partial<SimulationConfig>) => {
      const { simulation } = get();
      simulation.updateConfig(config);
      set({ config: { ...get().config, ...config } });
    },
    
    setPIDGains: (Kp: number, Ki: number, Kd: number) => {
      const { simulation } = get();
      simulation.setPIDGains(Kp, Ki, Kd);
      set({ pidGains: { Kp, Ki, Kd } });
    },
    
    setSetpoint: (roll: number, pitch: number, yaw: number) => {
      const { simulation } = get();
      simulation.setSetpoint(roll, pitch, yaw);
      set({ setpoint: { roll, pitch, yaw } });
    },
    
    setSensorNoise: (level: number) => {
      const { simulation } = get();
      simulation.setSensorNoise(level);
    },
    
    setInitialConditions: (wx: number, wy: number, wz: number) => {
      const { simulation } = get();
      simulation.setInitialConditions({ x: wx, y: wy, z: wz });
    },
    
    addLog: (message: string) => {
      set((state) => ({
        logs: [
          ...state.logs.slice(-49), // Keep last 50 logs
          { timestamp: new Date(), message },
        ],
      }));
    },
  };
});
