/**
 * sdrEngine.ts
 * SDR Digital Signal Processing Engine
 * Handles FFT, OOK demodulation, frame decoding for 2.4GHz S-Band
 */

// ─── FFT ────────────────────────────────────────────────────────────────────

/** Radix-2 Cooley-Tukey FFT (in-place, complex) */
export function fft(re: Float32Array, im: Float32Array): void {
  const n = re.length;
  // Bit-reversal permutation
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      [re[i], re[j]] = [re[j], re[i]];
      [im[i], im[j]] = [im[j], im[i]];
    }
  }
  // FFT butterfly
  for (let len = 2; len <= n; len <<= 1) {
    const half = len >> 1;
    const angle = (-2 * Math.PI) / len;
    const wRe = Math.cos(angle);
    const wIm = Math.sin(angle);
    for (let i = 0; i < n; i += len) {
      let curRe = 1, curIm = 0;
      for (let j = 0; j < half; j++) {
        const a = i + j, b = a + half;
        const tRe = curRe * re[b] - curIm * im[b];
        const tIm = curRe * im[b] + curIm * re[b];
        re[b] = re[a] - tRe;
        im[b] = im[a] - tIm;
        re[a] += tRe;
        im[a] += tIm;
        const nextRe = curRe * wRe - curIm * wIm;
        curIm = curRe * wIm + curIm * wRe;
        curRe = nextRe;
      }
    }
  }
}

/** Compute power spectral density in dB from time-domain samples */
export function computePSD(samples: Float32Array, fftSize: number): Float32Array {
  const re = new Float32Array(fftSize);
  const im = new Float32Array(fftSize);

  // Apply Hanning window
  for (let i = 0; i < fftSize; i++) {
    const w = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (fftSize - 1)));
    re[i] = (i < samples.length ? samples[i] : 0) * w;
    im[i] = 0;
  }

  fft(re, im);

  const psd = new Float32Array(fftSize / 2);
  for (let i = 0; i < fftSize / 2; i++) {
    const mag = re[i] * re[i] + im[i] * im[i];
    psd[i] = 10 * Math.log10(Math.max(mag / fftSize, 1e-12));
  }
  return psd;
}

// ─── Telemetry Parsing ──────────────────────────────────────────────────────

export interface TelemetryPacket {
  timestamp: number;
  x: number;
  y: number;
  z: number;
  temp: number;
  humidity: number;
  rssi: number;
  valid: boolean;
  raw?: string;
}

/** Parse RH_ASK telemetry frame: "X:123 Y:-45 Z:678 T:25.3 H:55.0" */
export function parseTelemetry(raw: string, rssi: number): TelemetryPacket {
  const pkt: TelemetryPacket = {
    timestamp: Date.now(),
    x: 0, y: 0, z: 0, temp: 0, humidity: 0,
    rssi,
    valid: false,
  };

  try {
    const xMatch = raw.match(/X:\s*(-?\d+)/);
    const yMatch = raw.match(/Y:\s*(-?\d+)/);
    const zMatch = raw.match(/Z:\s*(-?\d+)/);
    const tMatch = raw.match(/T:\s*(-?\d+\.?\d*)/);
    const hMatch = raw.match(/H:\s*(-?\d+\.?\d*)/);

    if (xMatch) pkt.x = parseInt(xMatch[1], 10);
    if (yMatch) pkt.y = parseInt(yMatch[1], 10);
    if (zMatch) pkt.z = parseInt(zMatch[1], 10);
    if (tMatch) pkt.temp = parseFloat(tMatch[1]);
    if (hMatch) pkt.humidity = parseFloat(hMatch[1]);

    // Strict: require ALL 5 fields for valid packet
    if (xMatch && yMatch && zMatch && tMatch && hMatch) {
      pkt.valid = true;
    }
  } catch { /* invalid frame */ }

  return pkt;
}

// ─── Signal Simulation ──────────────────────────────────────────────────────

/** Generate realistic 2.4 GHz band noise floor + signal */
export function generateSimulatedSignal(
  fftSize: number,
  centerFreqMHz: number,
  signalPresent: boolean,
  signalStrength: number,   // 0-1
  noiseFloor: number,       // dB e.g. -100
): Float32Array {
  const samples = new Float32Array(fftSize);
  const t = performance.now() / 1000;
  // Scale noise amplitude from dB noise floor
  const noiseAmp = Math.pow(10, noiseFloor / 40) * 0.5;
  // Carrier frequency offset based on center frequency deviation from 2400.0
  const freqOffset = (centerFreqMHz - 2400.0) * 0.1;

  for (let i = 0; i < fftSize; i++) {
    // White gaussian noise
    const u1 = Math.random();
    const u2 = Math.random();
    const gaussian = Math.sqrt(-2 * Math.log(Math.max(u1, 1e-10))) * Math.cos(2 * Math.PI * u2);
    samples[i] = gaussian * noiseAmp;

    if (signalPresent) {
      // OOK modulated carrier at center
      const carrier = Math.sin(2 * Math.PI * (0.25 + freqOffset) * i + t * 5);
      // OOK envelope — simulate bursts
      const burstPhase = (t * 2 + i / fftSize) % 1;
      const ook = burstPhase < 0.6 ? 1 : 0;
      samples[i] += carrier * ook * signalStrength * 0.15;

      // Add harmonics for realism
      samples[i] += Math.sin(2 * Math.PI * 0.5 * i + t * 3) * signalStrength * 0.03;
      samples[i] += Math.sin(2 * Math.PI * 0.125 * i + t * 7) * signalStrength * 0.02;
    }
  }

  return samples;
}

/** Generate a simulated telemetry string (matching Arduino format) */
export function generateSimulatedTelemetry(): string {
  const x = Math.floor(Math.random() * 1000 - 500);
  const y = Math.floor(Math.random() * 1000 - 500);
  const z = Math.floor(Math.random() * 1000 - 500);
  const temp = (Math.floor(Math.random() * 150 + 200) / 10).toFixed(1);
  const hum = (Math.floor(Math.random() * 600 + 300) / 10).toFixed(1);
  return `X:${x} Y:${y} Z:${z} T:${temp} H:${hum}`;
}

// ─── IQ Generation ──────────────────────────────────────────────────────────

export interface IQPoint { i: number; q: number; }

/** Generate OOK constellation points */
export function generateIQConstellation(signalPresent: boolean, count: number = 64): IQPoint[] {
  const points: IQPoint[] = [];
  for (let k = 0; k < count; k++) {
    if (signalPresent) {
      // OOK: two clusters — near origin (0) and at (1,0) with noise
      const isOn = Math.random() > 0.4;
      const noiseI = (Math.random() - 0.5) * 0.15;
      const noiseQ = (Math.random() - 0.5) * 0.15;
      points.push({
        i: (isOn ? 0.7 : 0) + noiseI,
        q: noiseQ,
      });
    } else {
      // Pure noise cluster around origin
      points.push({
        i: (Math.random() - 0.5) * 0.3,
        q: (Math.random() - 0.5) * 0.3,
      });
    }
  }
  return points;
}

// ─── Waterfall Helpers ──────────────────────────────────────────────────────

/** Map dB value to a hot colormap RGBA */
export function dbToColor(db: number, minDb: number, maxDb: number): [number, number, number] {
  const t = Math.max(0, Math.min(1, (db - minDb) / (maxDb - minDb)));

  if (t < 0.25) {
    // Black → Deep Blue
    const s = t / 0.25;
    return [0, 0, Math.floor(s * 140)];
  } else if (t < 0.5) {
    // Deep Blue → Cyan
    const s = (t - 0.25) / 0.25;
    return [0, Math.floor(s * 255), 140 + Math.floor(s * 115)];
  } else if (t < 0.75) {
    // Cyan → Yellow
    const s = (t - 0.5) / 0.25;
    return [Math.floor(s * 255), 255, Math.floor(255 - s * 255)];
  } else {
    // Yellow → Red → White
    const s = (t - 0.75) / 0.25;
    return [255, Math.floor(255 - s * 155), Math.floor(s * 200)];
  }
}

/** Smooth a PSD array with exponential moving average */
export function smoothPSD(
  current: Float32Array,
  previous: Float32Array | null,
  alpha: number = 0.3
): Float32Array {
  if (!previous) return current;
  const out = new Float32Array(current.length);
  for (let i = 0; i < current.length; i++) {
    out[i] = alpha * current[i] + (1 - alpha) * previous[i];
  }
  return out;
}
