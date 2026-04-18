/**
 * imuStore.ts
 * Zustand store for managing IMU sensor state and connection
 */

import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import { imuService, type IMUQuaternion } from '../services/IMUService';

interface IMUStore {
    // Connection state
    isConnected: boolean;

    // IMU data
    imuQuaternion: IMUQuaternion | null;
    environment: { temp: number; hum: number; alt: number } | null;
    motorValues: number[];

    dataRate: number; // Hz
    imuOffset: IMUQuaternion | null;

    // Mode toggle
    useIMUMode: boolean;
    motorsEnabled: boolean;

    // Actions
    connect: () => void;
    disconnect: () => void;
    toggleMode: () => void;
    toggleMotors: () => void;
    setUseIMUMode: (value: boolean) => void;
    setTargetAngles: (pitch: number, roll: number, yaw: number) => void;
    zeroIMU: () => void;
    calibrate: (mode: 'left' | 'right' | 'center' | 'stop') => void;
    rotateRelative: (axis: 'pitch' | 'roll' | 'yaw', degrees: number) => void;
}

export const useIMUStore = create<IMUStore>()(
    persist(
        (set, get) => {
            // Track data rate
            let lastTimestamp = 0;
            let frameCount = 0;
            let rateUpdateInterval: ReturnType<typeof setInterval> | null = null;

            // Subscribe to IMU service updates
            imuService.onTelemetry((data) => {
                set({
                    imuQuaternion: data.quaternion,
                    environment: data.environment,
                    motorValues: data.motor
                });
                frameCount++;
            });

            imuService.onStatusChange((connected) => {
                set({ isConnected: connected });

                if (connected) {
                    // Start tracking data rate
                    frameCount = 0;
                    lastTimestamp = Date.now();
                    rateUpdateInterval = setInterval(() => {
                        const elapsed = (Date.now() - lastTimestamp) / 1000;
                        const rate = elapsed > 0 ? frameCount / elapsed : 0;
                        set({ dataRate: Math.round(rate) });
                        frameCount = 0;
                        lastTimestamp = Date.now();
                    }, 1000);
                } else {
                    // Stop tracking
                    if (rateUpdateInterval) {
                        clearInterval(rateUpdateInterval);
                        rateUpdateInterval = null;
                    }
                    set({ dataRate: 0 });
                }
            });

            return {
                isConnected: false,
                imuQuaternion: null,
                imuOffset: null,
                environment: null,
                motorValues: [],
                dataRate: 0,
                useIMUMode: false,
                motorsEnabled: true, // Default ON for mission readiness

                connect: () => {
                    imuService.connect();
                },

                disconnect: () => {
                    imuService.disconnect();
                    set({
                        isConnected: false,
                        imuQuaternion: null,
                        imuOffset: null,
                        dataRate: 0,
                        motorsEnabled: false
                    });
                },

                toggleMode: () => {
                    const nextMode = !get().useIMUMode;
                    set({ useIMUMode: nextMode });
                    if (nextMode && !get().isConnected) {
                        imuService.connect();
                    }
                },

                toggleMotors: () => {
                    const newStatus = !get().motorsEnabled;
                    set({ motorsEnabled: newStatus });
                    imuService.setMotorPower(newStatus);
                },

                setUseIMUMode: (value: boolean) => {
                    if (value && !get().isConnected) {
                        imuService.connect();
                    }
                    set({ useIMUMode: value });
                },

                setTargetAngles: (pitch: number, roll: number, yaw: number) => {
                    imuService.setTargetAngles(pitch, roll, yaw);
                },

                zeroIMU: () => {
                    const quat = get().imuQuaternion;
                    if (quat) {
                        set({ imuOffset: quat });
                        console.log('[IMU] Offset set (Zeroed)');
                    }
                },

                calibrate: (mode: 'left' | 'right' | 'center' | 'stop') => {
                    imuService.sendCalibrate(mode);
                },

                rotateRelative: (axis: 'pitch' | 'roll' | 'yaw', degrees: number) => {
                    imuService.rotateRelative(axis, degrees);
                }
            };
        },
        {
            name: 'imu-storage',
            partialize: (state) => ({
                useIMUMode: state.useIMUMode,
                motorsEnabled: state.motorsEnabled
            }),
            onRehydrateStorage: (state) => {
                return (rehydratedState, error) => {
                    if (!error && rehydratedState?.useIMUMode) {
                        console.log('[IMU] Rehydrated: Auto-connecting to bridge...');
                        imuService.connect();
                    }
                }
            }
        }
    )
);
