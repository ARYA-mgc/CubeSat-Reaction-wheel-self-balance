/**
 * IMUService.ts
 * WebSocket client that connects to the UDP bridge server
 * and receives quaternion data from the BNO08X IMU sensor via ESP32
 */

export interface IMUQuaternion {
    i: number;
    j: number;
    k: number;
    real: number;
}

export interface EnvironmentData {
    temp: number;
    hum: number;
    alt: number;
}

export interface TelemetryPayload {
    quaternion: IMUQuaternion;
    environment: EnvironmentData;
    motor: number[];
    timestamp: number;
}

type TelemetryCallback = (data: TelemetryPayload) => void;
type StatusCallback = (connected: boolean) => void;

class IMUService {
    private ws: WebSocket | null = null;
    private reconnectTimeout: ReturnType<typeof setTimeout> | null = null;
    private readonly wsUrl: string;
    private readonly reconnectDelay = 2000; // ms
    private shouldReconnect = false;

    private telemetryCallbacks: Set<TelemetryCallback> = new Set();
    private statusCallbacks: Set<StatusCallback> = new Set();

    constructor(wsUrl: string = 'ws://localhost:8765') {
        this.wsUrl = wsUrl;
    }

    /**
     * Connect to the WebSocket bridge server
     */
    connect(): void {
        if (this.ws?.readyState === WebSocket.OPEN) {
            console.log('[IMU] Already connected');
            return;
        }

        this.shouldReconnect = true;
        this.attemptConnection();
    }

    private attemptConnection(): void {
        try {
            console.log('[IMU] Connecting to', this.wsUrl);
            this.ws = new WebSocket(this.wsUrl);

            this.ws.onopen = () => {
                console.log('[IMU] Connected to bridge server');
                this.notifyStatus(true);
            };

            this.ws.onmessage = (event) => {
                try {
                    const msg = JSON.parse(event.data);

                    // Handle new format: { type: 'telemetry', payload: ... }
                    if (msg.type === 'telemetry' && msg.payload) {
                        if (Math.random() < 0.05) {
                            console.log('[IMU] Telemetry Received - Lag:', Date.now() - msg.payload.timestamp, 'ms');
                        }
                        this.notifyTelemetry(msg.payload);
                    }
                    // Handle legacy direct quaternion format (fallback)
                    else if (msg.i !== undefined) {
                        this.notifyTelemetry({
                            quaternion: msg,
                            environment: { temp: 0, hum: 0, alt: 0 },
                            motor: [0, 0, 0],
                            timestamp: Date.now()
                        });
                    }
                } catch (e) {
                    console.error('[IMU] Failed to parse data:', e);
                }
            };

            this.ws.onclose = () => {
                console.log('[IMU] Disconnected from bridge server');
                this.notifyStatus(false);
                this.scheduleReconnect();
            };

            this.ws.onerror = (error) => {
                console.error('[IMU] WebSocket error - Check if Bridge is running on localhost:8765:', error);
            };
        } catch (error) {
            console.error('[IMU] Connection failed:', error);
            this.scheduleReconnect();
        }
    }

    private scheduleReconnect(): void {
        if (this.shouldReconnect && !this.reconnectTimeout) {
            this.reconnectTimeout = setTimeout(() => {
                this.reconnectTimeout = null;
                if (this.shouldReconnect) {
                    console.log('[IMU] Attempting reconnection...');
                    this.attemptConnection();
                }
            }, this.reconnectDelay);
        }
    }

    /**
     * Disconnect from the WebSocket bridge server
     */
    disconnect(): void {
        this.shouldReconnect = false;

        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
            this.reconnectTimeout = null;
        }

        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }

        this.notifyStatus(false);
        console.log('[IMU] Disconnected');
    }

    /**
     * Check if currently connected
     */
    isConnected(): boolean {
        return this.ws?.readyState === WebSocket.OPEN;
    }

    /**
     * Subscribe to telemetry updates
     */
    onTelemetry(callback: TelemetryCallback): () => void {
        this.telemetryCallbacks.add(callback);
        return () => this.telemetryCallbacks.delete(callback);
    }

    /**
     * Subscribe to connection status changes
     */
    onStatusChange(callback: StatusCallback): () => void {
        this.statusCallbacks.add(callback);
        return () => this.statusCallbacks.delete(callback);
    }

    /**
     * Send target orientation commands to the ESP32
     */
    setTargetAngles(pitch: number, roll: number, yaw: number): void {
        if (this.ws?.readyState === WebSocket.OPEN) {
            const cmd = {
                type: 'control',
                payload: {
                    pitch: pitch,
                    roll: roll,
                    yaw: yaw
                }
            };
            this.ws.send(JSON.stringify(cmd));
            console.log('[IMU] Sent target angles:', { pitch, roll, yaw });
        }
    }

    /**
     * Send specialized rotation command (relative)
     */
    rotateRelative(axis: 'pitch' | 'roll' | 'yaw', degrees: number): void {
        if (this.ws?.readyState === WebSocket.OPEN) {
            const cmd = {
                type: 'rotate_rel',
                payload: {
                    axis: axis,
                    degrees: degrees
                }
            };
            this.ws.send(JSON.stringify(cmd));
            console.log(`[IMU] Sent relative rotation: ${degrees}° on ${axis}`);
        }
    }

    /**
     * Send calibration command
     */
    sendCalibrate(mode: 'left' | 'right' | 'center' | 'stop'): void {
        if (this.ws?.readyState === WebSocket.OPEN) {
            const cmd = {
                type: 'calibrate',
                payload: {
                    mode: mode
                }
            };
            this.ws.send(JSON.stringify(cmd));
            console.log(`[IMU] Sent calibration command: ${mode}`);
        }
    }

    private notifyTelemetry(data: TelemetryPayload): void {
        this.telemetryCallbacks.forEach(cb => cb(data));
    }

    /**
     * Set master motor power state
     */
    setMotorPower(enabled: boolean): void {
        if (this.ws?.readyState === WebSocket.OPEN) {
            const cmd = {
                type: 'motor_toggle',
                payload: {
                    enabled: enabled
                }
            };
            this.ws.send(JSON.stringify(cmd));
            console.log(`[IMU] Sent motor toggle: ${enabled ? 'ENABLED' : 'DISABLED'}`);
        }
    }

    private notifyStatus(connected: boolean): void {
        this.statusCallbacks.forEach(cb => cb(connected));
    }
}

// Singleton instance
export const imuService = new IMUService();
