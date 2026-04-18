/**
 * sdrStore.ts — v3
 * Zustand store for SDR state — real RTL-SDR via WebSocket (binary + JSON)
 */

import { create } from 'zustand';
import type { TelemetryPacket } from '../simulation/sdrEngine';

export interface SDRState {
    // Connection
    mode: 'simulated' | 'usrp';
    isReceiving: boolean;
    serverConnected: boolean;
    rtlAvailable: boolean;
    serverMessage: string;

    // Tuning
    centerFreqMHz: number;
    sampleRateKHz: number;
    gain: number;
    squelch: number;

    // Signal metrics
    snr: number;
    rssi: number;
    packetCount: number;
    errorCount: number;
    bitrate: number;
    bitsDecoded: number;
    peakPower: number;

    // Data
    packets: TelemetryPacket[];
    waterfallHistory: Float32Array[];
    latestPSD: Float32Array | null;
    iqPoints: { i: number; q: number }[];
    hwInfo: {
        model: string;
        serial: string;
        mboard_name: string;
        master_clock_rate: number;
    } | null;

    // WebSocket
    ws: WebSocket | null;

    // Actions
    setMode: (mode: 'simulated' | 'usrp') => void;
    toggleReceiving: () => void;
    startReceiving: () => void;
    stopReceiving: () => void;
    setCenterFreq: (freq: number) => void;
    setSampleRate: (rate: number) => void;
    setGain: (gain: number) => void;
    setSquelch: (sq: number) => void;
    addPacket: (pkt: TelemetryPacket) => void;
    pushWaterfallRow: (row: Float32Array) => void;
    setLatestPSD: (psd: Float32Array) => void;
    updateMetrics: (m: { snr: number; rssi: number; bitrate: number; bits?: number; peak?: number }) => void;
    incrementError: () => void;
    setIQPoints: (pts: { i: number; q: number }[]) => void;
    connectServer: () => void;
    disconnectServer: () => void;
    setServerMessage: (msg: string) => void;
    reset: () => void;
}

const MAX_PACKETS = 200;
const MAX_WATERFALL = 200;
const SDR_WS = 'ws://localhost:8766';

export const useSDRStore = create<SDRState>((set, get) => ({
    mode: 'simulated',
    isReceiving: false,
    serverConnected: false,
    rtlAvailable: false,
    serverMessage: '',

    centerFreqMHz: 2400.0,
    sampleRateKHz: 250,
    gain: 40,
    squelch: -100,

    snr: 0, rssi: -90, packetCount: 0, errorCount: 0, bitrate: 2000, bitsDecoded: 0, peakPower: 0,

    packets: [],
    waterfallHistory: [],
    latestPSD: null,
    iqPoints: [],
    hwInfo: null,
    ws: null,

    setMode: (mode) => set({ mode }),

    toggleReceiving: () => { const s = get(); s.isReceiving ? s.stopReceiving() : s.startReceiving(); },

    startReceiving: () => {
        const s = get();
        if (s.mode === 'usrp' && s.ws?.readyState === WebSocket.OPEN) {
            s.ws.send(JSON.stringify({
                action: 'start',
                frequency: Math.round(s.centerFreqMHz * 1e6),
                gain: s.gain,
                sampleRate: s.sampleRateKHz * 1000,
            }));
        }
        set({ isReceiving: true });
    },

    stopReceiving: () => {
        const s = get();
        if (s.mode === 'usrp' && s.ws?.readyState === WebSocket.OPEN) {
            s.ws.send(JSON.stringify({ action: 'stop' }));
        }
        set({ isReceiving: false });
    },

    setCenterFreq: (freq) => {
        set({ centerFreqMHz: freq });
        const s = get();
        if (s.mode === 'usrp' && s.ws?.readyState === WebSocket.OPEN && s.isReceiving) {
            s.ws.send(JSON.stringify({ action: 'setFreq', frequency: Math.round(freq * 1e6) }));
        }
    },

    setSampleRate: (rate) => {
        set({ sampleRateKHz: rate });
        const s = get();
        if (s.mode === 'usrp' && s.ws?.readyState === WebSocket.OPEN && s.isReceiving) {
            s.ws.send(JSON.stringify({ action: 'setRate', sampleRate: rate * 1000 }));
        }
    },

    setGain: (gain) => {
        set({ gain });
        const s = get();
        if (s.mode === 'usrp' && s.ws?.readyState === WebSocket.OPEN && s.isReceiving) {
            s.ws.send(JSON.stringify({ action: 'setGain', gain }));
        }
    },

    setSquelch: (sq) => set({ squelch: sq }),

    addPacket: (pkt) => set(s => ({
        packets: [...s.packets.slice(-(MAX_PACKETS - 1)), pkt],
        packetCount: s.packetCount + 1,
    })),

    pushWaterfallRow: (row) => set(s => ({
        waterfallHistory: [...s.waterfallHistory.slice(-(MAX_WATERFALL - 1)), row],
    })),

    setLatestPSD: (psd) => set({ latestPSD: psd }),

    updateMetrics: (m) => set({
        snr: m.snr, rssi: m.rssi, bitrate: m.bitrate,
        bitsDecoded: m.bits ?? 0, peakPower: m.peak ?? 0,
    }),

    incrementError: () => set(s => ({ errorCount: s.errorCount + 1 })),
    setIQPoints: (pts) => set({ iqPoints: pts }),
    setServerMessage: (msg) => set({ serverMessage: msg }),

    connectServer: () => {
        const state = get();
        if (state.ws) state.ws.close();

        const ws = new WebSocket(SDR_WS);
        ws.binaryType = 'arraybuffer'; // Required for binary spectrum data

        ws.onopen = () => {
            console.log('[SDR] Connected');
            set({ serverConnected: true, serverMessage: 'Connected to SDR server' });
        };
        ws.onclose = () => {
            console.log('[SDR] Disconnected');
            set({
                serverConnected: false,
                ws: null,
                serverMessage: 'Disconnected',
                latestPSD: null,
                waterfallHistory: [],
                isReceiving: false
            });
        };
        ws.onerror = () => {
            set({
                serverConnected: false,
                serverMessage: 'Server Connection Error — Is the Bridge running?',
                latestPSD: null,
                waterfallHistory: [],
                isReceiving: false
            });
        };

        ws.onmessage = (event) => {
            const store = get();

            // ── Binary message: spectrum PSD ──
            if (event.data instanceof ArrayBuffer) {
                const view = new Uint8Array(event.data);
                if (view[0] === 0x01 && view.length > 1) {
                    // Spectrum: 1 byte marker + Float32Array
                    const psd = new Float32Array(event.data.slice(1));
                    store.setLatestPSD(psd);
                    store.pushWaterfallRow(psd);
                }
                return;
            }

            // ── JSON messages ──
            try {
                const msg = JSON.parse(event.data as string);
                switch (msg.type) {
                    case 'iq':
                        store.setIQPoints(msg.points || []);
                        break;
                    case 'metrics':
                        store.updateMetrics({
                            snr: msg.snr, rssi: msg.rssi, bitrate: msg.bitrate,
                            bits: msg.bits, peak: msg.peak
                        });
                        break;
                    case 'packet':
                        store.addPacket(msg.data);
                        if (!msg.data?.valid) store.incrementError();
                        break;
                    case 'hw_info':
                        set({
                            hwInfo: {
                                model: msg.model || 'Unknown',
                                serial: msg.serial || 'Unknown',
                                mboard_name: msg.mboard_name || 'USRP',
                                master_clock_rate: msg.master_clock_rate || 0,
                            }
                        });
                        break;
                    case 'status':
                        set({
                            rtlAvailable: msg.usrpAvailable ?? msg.rtlAvailable ?? false,
                            isReceiving: msg.capturing ?? false,
                        });
                        if (msg.frequency) set({ centerFreqMHz: msg.frequency / 1e6 });
                        if (msg.gain !== undefined) set({ gain: msg.gain });

                        // If stopped, clear data to show "Acquiring..." or "Stopped"
                        if (msg.capturing === false) {
                            set({ latestPSD: null, waterfallHistory: [] });
                        }
                        break;
                    case 'error':
                        set({
                            serverMessage: msg.message || 'Error',
                            isReceiving: false,
                            latestPSD: null,
                            waterfallHistory: [],
                            snr: 0,
                            rssi: -120
                        });
                        break;
                    case 'clear':
                        set({
                            latestPSD: null, waterfallHistory: [], iqPoints: [],
                            snr: 0, rssi: -120, isReceiving: false, bitsDecoded: 0, peakPower: 0,
                        });
                        break;
                }
            } catch (e) {
                console.error('[SDR] Parse error:', e);
            }
        };

        set({ ws });
    },

    disconnectServer: () => {
        const s = get();
        if (s.ws) { s.ws.close(); set({ ws: null, serverConnected: false }); }
    },

    reset: () => {
        const s = get();
        if (s.ws && s.isReceiving) s.ws.send(JSON.stringify({ action: 'stop' }));
        set({
            isReceiving: false, snr: 0, rssi: -90, packetCount: 0, errorCount: 0,
            packets: [], waterfallHistory: [], latestPSD: null, iqPoints: [],
            serverMessage: '', bitsDecoded: 0, peakPower: 0,
        });
    },
}));
