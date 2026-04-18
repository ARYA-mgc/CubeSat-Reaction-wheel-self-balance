/**
 * SDR Bridge Server — v10 (Adaptive Bitrate & Clock Recovery)
 * 
 * CHANGES vs v9:
 * 1. Adaptive Bitrate: Calculates real 'samples per bit' (SPB) from
 *    the median of shortest pulses in the buffer. Corrects clock drift.
 * 2. Glitch Filter: Removes tiny pulses (< 0.5 bits) that confuse decoding.
 * 3. Enhanced Fuzzy Repair: fixes more common corruption patterns.
 */

const { spawn, execSync } = require('child_process');
const WebSocket = require('ws');
const fs = require('fs');

// ═══════════════════════════════════════════════════════════════
// CONFIG
// ═══════════════════════════════════════════════════════════════
const WS_PORT = 8766;
const DEFAULT_FREQ = 433852000;  // 433.852 MHz (User Specific)
const DEFAULT_RATE = 250000;
const FFT_SIZE = 1024;
const DEFAULT_GAIN = 40;
const SPECTRUM_INTERVAL = 60;
const IQ_INTERVAL = 150;
const OOK_ENV_SMOOTH = 8;         // Envelope smoothing factor (samples)
const DECODE_WINDOW_MS = 500;     // 0.5s decode window (faster response)
const TARGET_BITRATE = 2000;      // RadioHead RH_ASK default

// ═══════════════════════════════════════════════════════════════
// RTL-SDR LOCATE
// ═══════════════════════════════════════════════════════════════
const SEARCH_PATHS = [
    'C:\\Users\\visha\\Downloads\\Release\\x64\\rtl_sdr.exe',
    'C:\\Program Files\\rtl-sdr\\rtl_sdr.exe',
    'C:\\rtlsdr\\rtl_sdr.exe',
];
let _rtlPath = null;
function findRtlSdr() {
    if (_rtlPath) return _rtlPath;
    for (const p of SEARCH_PATHS) { if (fs.existsSync(p)) { _rtlPath = p; return p; } }
    try {
        const r = execSync('where rtl_sdr.exe 2>nul', { timeout: 3000, stdio: 'pipe' });
        const found = r.toString().trim().split('\n')[0].trim();
        if (found && fs.existsSync(found)) { _rtlPath = found; return found; }
    } catch (_) { }
    return null;
}

// ... (existing code omitted) ...



// ═══════════════════════════════════════════════════════════════
// DSP UTILS
// ═══════════════════════════════════════════════════════════════
function fft(re, im) {
    const n = re.length;
    for (let i = 1, j = 0; i < n; i++) {
        let bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { [re[i], re[j]] = [re[j], re[i]];[im[i], im[j]] = [im[j], im[i]]; }
    }
    for (let len = 2; len <= n; len <<= 1) {
        const half = len >> 1, ang = -2 * Math.PI / len;
        const wR = Math.cos(ang), wI = Math.sin(ang);
        for (let i = 0; i < n; i += len) {
            let cR = 1, cI = 0;
            for (let j = 0; j < half; j++) {
                const a = i + j, b = a + half;
                const tR = cR * re[b] - cI * im[b], tI = cR * im[b] + cI * re[b];
                re[b] = re[a] - tR; im[b] = im[a] - tI;
                re[a] += tR; im[a] += tI;
                const nR = cR * wR - cI * wI; cI = cR * wI + cI * wR; cR = nR;
            }
        }
    }
}

function computePSD(buf, off, N) {
    const re = new Float64Array(N), im = new Float64Array(N);
    for (let i = 0; i < N; i++) {
        const I = (buf[off + i * 2] - 127.5) / 127.5;
        const Q = (buf[off + i * 2 + 1] - 127.5) / 127.5;
        const w = 0.5 * (1 - Math.cos(2 * Math.PI * i / (N - 1)));
        re[i] = I * w; im[i] = Q * w;
    }
    fft(re, im);
    const psd = new Float32Array(N);
    for (let i = 0; i < N; i++) {
        const j = (i + N / 2) % N;
        psd[i] = 10 * Math.log10(Math.max(re[j] * re[j] + im[j] * im[j], 1e-12) / N);
    }
    return psd;
}

function lowPassFilterIQ(buf, off, n, sampleRate, cutoffHz) {
    const win = Math.max(2, Math.round(sampleRate / (2 * cutoffHz)));
    const filtI = new Float64Array(n), filtQ = new Float64Array(n);
    let sumI = 0, sumQ = 0;
    for (let i = 0; i < n; i++) {
        sumI += buf[off + i * 2] - 127.5;
        sumQ += buf[off + i * 2 + 1] - 127.5;
        if (i >= win) {
            sumI -= buf[off + (i - win) * 2] - 127.5;
            sumQ -= buf[off + (i - win) * 2 + 1] - 127.5;
        }
        const c = Math.min(i + 1, win);
        filtI[i] = sumI / c;
        filtQ[i] = sumQ / c;
    }
    return { filtI, filtQ };
}

function lowPassFilterArrays(iIn, qIn, sampleRate, cutoffHz) {
    const n = iIn.length;
    const win = Math.max(2, Math.round(sampleRate / (2 * cutoffHz)));
    const filtI = new Float64Array(n), filtQ = new Float64Array(n);
    let sumI = 0, sumQ = 0;

    for (let i = 0; i < n; i++) {
        sumI += iIn[i];
        sumQ += qIn[i];
        if (i >= win) {
            sumI -= iIn[i - win];
            sumQ -= qIn[i - win];
        }
        const c = Math.min(i + 1, win);
        filtI[i] = sumI / c;
        filtQ[i] = sumQ / c;
    }
    return { filtI, filtQ };
}

// ═══════════════════════════════════════════════════════════════
// DC BLOCKER (IIR Highpass)
// ═══════════════════════════════════════════════════════════════
function removeDC(buf, off, n) {
    const iOut = new Float64Array(n);
    const qOut = new Float64Array(n);
    let prevX_I = 0, prevY_I = 0;
    let prevX_Q = 0, prevY_Q = 0;
    const alpha = 0.99;

    for (let k = 0; k < n; k++) {
        const I = buf[off + k * 2] - 127.5;
        const Q = buf[off + k * 2 + 1] - 127.5;

        // y[n] = alpha * (y[n-1] + x[n] - x[n-1])
        const yI = alpha * (prevY_I + I - prevX_I);
        const yQ = alpha * (prevY_Q + Q - prevX_Q);

        iOut[k] = yI;
        qOut[k] = yQ;

        prevX_I = I; prevY_I = yI;
        prevX_Q = Q; prevY_Q = yQ;
    }
    return { iOut, qOut };
}

function envelopeIQ(filtI, filtQ) {
    const n = filtI.length, env = new Float64Array(n);
    for (let i = 0; i < n; i++) env[i] = Math.sqrt(filtI[i] * filtI[i] + filtQ[i] * filtQ[i]);
    return env;
}

function smoothEnv(env, win) {
    const w = Math.max(1, win), o = new Float64Array(env.length);
    let s = 0;
    for (let i = 0; i < env.length; i++) {
        s += env[i]; if (i >= w) s -= env[i - w];
        o[i] = s / Math.min(i + 1, w);
    }
    return o;
}

// ═══════════════════════════════════════════════════════════════
// PULSE DECODING & CLOCK RECOVERY
// ═══════════════════════════════════════════════════════════════
function adaptiveThreshold(env) {
    const sorted = env.slice().sort((a, b) => a - b);
    const p10 = sorted[Math.floor(sorted.length * 0.1)];
    const p90 = sorted[Math.floor(sorted.length * 0.9)];
    const range = p90 - p10;
    return {
        high: p10 + range * 0.50, // Lower threshold to catch weak edges
        low: p10 + range * 0.25,
        min: p10, max: p90, range
    };
}

function extractPulses(env, thrHigh, thrLow) {
    const pulses = [];
    let state = 0;
    let count = 0;
    if (env[0] > thrHigh) state = 1;

    for (let i = 0; i < env.length; i++) {
        if (state === 0 && env[i] > thrHigh) {
            if (count > 0) pulses.push({ level: 0, samples: count });
            state = 1; count = 1;
        } else if (state === 1 && env[i] < thrLow) {
            if (count > 0) pulses.push({ level: 1, samples: count });
            state = 0; count = 1;
        } else {
            count++;
        }
    }
    if (count > 0) pulses.push({ level: state, samples: count });
    return pulses;
}

/**
 * ADAPTIVE BITRATE ESTIMATION
 * Finds the median width of "short" pulses to determine SPB.
 */
function estimateSPB(pulses, nominalSPB) {
    // Filter glitches
    const valid = pulses.filter(p => p.samples > nominalSPB * 0.3);
    if (valid.length < 5) return nominalSPB;

    // Sort by duration
    valid.sort((a, b) => a.samples - b.samples);

    // Take the bottom 50% (short pulses)
    const shorts = valid.slice(0, Math.ceil(valid.length * 0.5));

    // Median of shorts
    const median = shorts[Math.floor(shorts.length / 2)].samples;

    // Sanity check: must be within 50% of nominal
    if (median > nominalSPB * 0.5 && median < nominalSPB * 1.5) {
        return median;
    }
    return nominalSPB;
}

function pulsesToBits(pulses, spb) {
    const bits = [];
    for (const p of pulses) {
        if (p.samples < spb * 0.3) continue; // Glitch rejection
        let n = Math.round(p.samples / spb);
        if (n < 1) n = 1;
        if (n > 40) n = 40;
        for (let i = 0; i < n; i++) bits.push(p.level);
    }
    return bits;
}

function invertBits(bits) { return bits.map(b => b === 1 ? 0 : 1); }

/**
 * Compute envelope directly from raw IQ buffer (no LPF).
 * This preserves the signal regardless of carrier frequency offset.
 */
function rawEnvelope(buf, off, nSamples) {
    const env = new Float64Array(nSamples);
    for (let i = 0; i < nSamples; i++) {
        const I = buf[off + i * 2] - 127.5;
        const Q = buf[off + i * 2 + 1] - 127.5;
        env[i] = Math.sqrt(I * I + Q * Q);
    }
    return env;
}

// ═══════════════════════════════════════════════════════════════
// RADIOHEAD RH_ASK FRAME DECODER (Protocol-aware)
// ═══════════════════════════════════════════════════════════════

// RadioHead 4-to-6 encoding table
const S46 = [0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34];
const S64 = new Array(64).fill(-1);
for (let i = 0; i < 16; i++) S64[S46[i]] = i;

// RH_ASK start symbol: 0xB38 = 101100111000 (12 bits)
// This pattern CANNOT appear in valid 4-to-6 data (0x38 is not a valid symbol)
const START_SYMBOL = [1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0];
const START_INVERTED = [0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1];

// Duplicate suppression
let lastDecodedRaw = '';
let lastDecodedTime = 0;
const DEDUP_WINDOW_MS = 500;

/**
 * CRC-CCITT update — matches RadioHead's _crc_ccitt_update exactly
 */
function crcUpdate(crc, data) {
    data = (data ^ (crc & 0xFF)) & 0xFF;
    data = (data ^ ((data << 4) & 0xFF)) & 0xFF;
    return (((data << 8) & 0xFFFF) | ((crc >> 8) & 0xFF)) ^ ((data >> 4) & 0xFF) ^ (((data << 3) & 0xFFFF));
}

function computeCRC(bytes, start, end) {
    let crc = 0xFFFF;
    for (let i = start; i < end; i++) {
        crc = crcUpdate(crc, bytes[i]) & 0xFFFF;
    }
    return crc;
}

/**
 * Decode 4-to-6 encoded bytes from bit stream starting at position `pos`.
 * Returns array of decoded bytes, stops at first invalid symbol pair.
 */
function decode4to6(bits, pos, maxBytes) {
    const bytes = [];
    for (let i = pos; bytes.length < maxBytes && (i + 12) <= bits.length; i += 12) {
        let symHi = 0, symLo = 0;
        for (let k = 0; k < 6; k++) {
            symHi = (symHi << 1) | bits[i + k];
            symLo = (symLo << 1) | bits[i + 6 + k];
        }
        const hi = S64[symHi];
        const lo = S64[symLo];
        if (hi >= 0 && lo >= 0) {
            bytes.push((hi << 4) | lo);
        } else {
            break; // End of valid data
        }
    }
    return bytes;
}

/**
 * Search for the RH_ASK start symbol pattern in the bit stream.
 * Returns positions immediately AFTER the start symbol.
 * Allows up to `maxErrors` bit mismatches for fuzzy matching.
 */
function findStartSymbols(bits, pattern, maxErrors) {
    const positions = [];
    for (let i = 0; i <= bits.length - pattern.length; i++) {
        let errors = 0;
        for (let j = 0; j < pattern.length; j++) {
            if (bits[i + j] !== pattern[j]) {
                errors++;
                if (errors > maxErrors) break;
            }
        }
        if (errors <= maxErrors) {
            positions.push(i + pattern.length);
        }
    }
    return positions;
}

/**
 * Try to decode a RadioHead frame at a given bit position.
 * RadioHead frame: [len] [to] [from] [id] [flags] [payload...] [crc_lo] [crc_hi]
 * len = total bytes including len, header, payload, and CRC.
 *
 * Returns decoded result or null.
 */
function tryDecodeFrame(bits, dataStart) {
    // Decode up to 80 bytes
    const bytes = decode4to6(bits, dataStart, 80);
    if (bytes.length < 7) return null; // Minimum: len(1) + header(4) + crc(2)

    const frameLen = bytes[0];
    if (frameLen < 7 || frameLen > 70) return null; // Sanity check
    if (bytes.length < frameLen) return null; // Not enough decoded bytes

    // CRC verification: computed over bytes[0..frameLen-3], compared with bytes[frameLen-2..frameLen-1]
    const crcComputed = computeCRC(bytes, 0, frameLen - 2);
    const crcReceived = (bytes[frameLen - 2]) | (bytes[frameLen - 1] << 8);
    const crcOk = (crcComputed === crcReceived);

    // Extract payload: bytes 5 through frameLen-3 (skip len,to,from,id,flags; exclude CRC)
    const payloadBytes = bytes.slice(5, frameLen - 2);
    let payload = '';
    for (const b of payloadBytes) {
        if (b >= 32 && b <= 126) {
            payload += String.fromCharCode(b);
        }
    }

    if (payload.length < 3) return null;

    return {
        raw: payload,
        len: payload.length,
        crcOk,
        to: bytes[1],
        from: bytes[2],
        id: bytes[3],
        flags: bytes[4],
        frameLen
    };
}

/**
 * Brute-force scan: try all 12 bit offsets, decode contiguous byte runs,
 * and look for valid telemetry patterns. Used as fallback when no start
 * symbol is found. Only accepts results with ALL 5 telemetry fields.
 */
function bruteForceScn(bits) {
    const results = [];
    for (let offset = 0; offset < 12; offset++) {
        const bytes = decode4to6(bits, offset, 80);
        if (bytes.length < 10) continue;

        // Convert to string
        let str = '';
        for (const b of bytes) {
            if (b >= 32 && b <= 126) str += String.fromCharCode(b);
        }

        // Strict match: must contain ALL 5 telemetry fields
        const fullMatch = str.match(/X:\s*-?\d+\s+Y:\s*-?\d+\s+Z:\s*-?\d+\s+T:\s*-?\d+\.?\d*\s+H:\s*-?\d+\.?\d*/);
        if (fullMatch) {
            results.push({ raw: fullMatch[0], len: fullMatch[0].length, crcOk: false, brute: true });
        }
    }
    return results;
}

/**
 * CRC-based frame scanning: scan all 12 bit offsets, decode contiguous
 * byte runs, then slide a window checking every position for a valid
 * RH_ASK frame (length byte + CRC match). No start symbol needed.
 * CRC provides 1:65536 false-positive protection.
 */
function crcFrameScan(bits) {
    const results = [];

    for (let offset = 0; offset < 12; offset++) {
        // Decode ALL contiguous valid 4-to-6 symbol pairs
        const allBytes = [];
        const byteRuns = []; // [[startIdx, bytes], ...]
        let runStart = 0;
        let run = [];

        for (let i = offset; i + 12 <= bits.length; i += 12) {
            let symHi = 0, symLo = 0;
            for (let k = 0; k < 6; k++) {
                symHi = (symHi << 1) | bits[i + k];
                symLo = (symLo << 1) | bits[i + 6 + k];
            }
            const hi = S64[symHi];
            const lo = S64[symLo];

            if (hi >= 0 && lo >= 0) {
                if (run.length === 0) runStart = allBytes.length;
                const byte = (hi << 4) | lo;
                allBytes.push(byte);
                run.push(byte);
            } else {
                if (run.length >= 7) {
                    byteRuns.push({ start: runStart, bytes: run.slice() });
                }
                allBytes.push(-1); // Invalid marker
                run = [];
            }
        }
        if (run.length >= 7) {
            byteRuns.push({ start: runStart, bytes: run.slice() });
        }

        // For each contiguous run, try every position as a potential frame start
        for (const { bytes } of byteRuns) {
            for (let j = 0; j < bytes.length - 6; j++) {
                const frameLen = bytes[j];
                if (frameLen < 7 || frameLen > 70) continue;
                if (j + frameLen > bytes.length) continue;

                // Verify CRC over bytes[j..j+frameLen-3]
                const crcComputed = computeCRC(bytes, j, j + frameLen - 2);
                const crcReceived = bytes[j + frameLen - 2] | (bytes[j + frameLen - 1] << 8);

                if (crcComputed === crcReceived) {
                    // CRC MATCH! Extract payload
                    const payloadBytes = bytes.slice(j + 5, j + frameLen - 2);
                    let payload = '';
                    for (const b of payloadBytes) {
                        if (b >= 32 && b <= 126) payload += String.fromCharCode(b);
                    }
                    if (payload.length >= 5) {
                        results.push({
                            raw: payload,
                            len: payload.length,
                            crcOk: true,
                            to: bytes[j + 1],
                            from: bytes[j + 2],
                            id: bytes[j + 3],
                            flags: bytes[j + 4],
                            frameLen,
                            method: `crc_scan_off${offset}`
                        });
                    }
                }
            }
        }
    }
    return results;
}

/**
 * Raw text scan: for transmitters that don't use 4-to-6 encoding at all.
 * Tries to find the telemetry ASCII pattern directly in the raw bit stream
 * by decoding as raw 8-bit bytes (no 4-to-6).
 */
function rawTextScan(bits) {
    const results = [];

    for (let offset = 0; offset < 8; offset++) {
        let str = '';
        for (let i = offset; i + 8 <= bits.length; i += 8) {
            let byte = 0;
            for (let k = 0; k < 8; k++) {
                byte = (byte << 1) | bits[i + k];
            }
            if (byte >= 32 && byte <= 126) {
                str += String.fromCharCode(byte);
            }
        }

        if (str.length < 15) continue;

        const fullMatch = str.match(/X:\s*-?\d+\s+Y:\s*-?\d+\s+Z:\s*-?\d+\s+T:\s*-?\d+\.?\d*\s+H:\s*-?\d+\.?\d*/);
        if (fullMatch) {
            results.push({ raw: fullMatch[0], len: fullMatch[0].length, crcOk: false, method: `raw8_off${offset}` });
        }
    }
    return results;
}

/**
 * Main packet scanner: 4 phases of detection.
 */
function scanForPackets(bits) {
    const candidates = [];

    // === PHASE 1: Start symbol detection (exact + fuzzy with CRC) ===
    const startPositions = findStartSymbols(bits, START_SYMBOL, 0);
    for (const pos of startPositions) {
        const frame = tryDecodeFrame(bits, pos);
        if (frame) candidates.push({ ...frame, method: 'start_exact' });
    }
    const startInvPositions = findStartSymbols(bits, START_INVERTED, 0);
    for (const pos of startInvPositions) {
        const frame = tryDecodeFrame(bits, pos);
        if (frame) candidates.push({ ...frame, method: 'start_exact_inv' });
    }

    if (candidates.length === 0) {
        for (const maxErr of [1, 2]) {
            for (const [pattern, label] of [[START_SYMBOL, 'fuzzy'], [START_INVERTED, 'fuzzy_inv']]) {
                const positions = findStartSymbols(bits, pattern, maxErr);
                for (const pos of positions) {
                    const frame = tryDecodeFrame(bits, pos);
                    if (frame && frame.crcOk) {
                        candidates.push({ ...frame, method: `${label}_${maxErr}` });
                    }
                }
            }
            if (candidates.length > 0) break;
        }
    }

    // === PHASE 5: SMART SYMBOL SEARCH (New in v23) ===
    // Scan for ANY run of valid 4-to-6 symbols, ignore start symbol
    if (candidates.length === 0) {
        for (let offset = 0; offset < 12; offset++) {
            let validRun = 0;
            let startIdx = -1;
            // Iterate bits with step of 6 (one symbol)
            for (let i = offset; i + 6 <= bits.length; i += 6) {
                let symbol = 0;
                for (let k = 0; k < 6; k++) symbol = (symbol << 1) | bits[i + k];

                if (S64[symbol] !== -1) { // Is it a valid RH symbol?
                    if (validRun === 0) startIdx = i;
                    validRun++;
                } else {
                    if (validRun >= 8) { // Found 8 valid symbols in a row (4 bytes)
                        // Try to decode from startIdx
                        // We need to find the LENGTH byte potentially?
                        // Or just decode as much as possible
                        const frameBytes = decode4to6(bits, startIdx, validRun); // decode validRun nibbles = validRun/2 bytes
                        // Scan the decoded bytes for ANY telemetry pattern ("X:", "Y:", "T:", etc.)
                        let s = '';
                        for (const b of frameBytes) {
                            if (b >= 32 && b <= 126) s += String.fromCharCode(b);
                            else s += `\\x${b.toString(16).padStart(2, '0').toUpperCase()}`;
                        }

                        // v28: Deep Debug Log
                        // Only log if we have something interesting (e.g. T:, H:, or just long valid run)
                        if (validRun >= 8) console.log(`[v28-SCAN] off=${offset} len=${validRun} s="${s}"`);

                        // v26: Relaxed check - find any field
                        if (/[XYZTH]:/.test(s)) {
                            console.log(`[v28-MATCH] Regex matched! s="${s}"`);
                            const fullMatch = s.match(/(?:X|Y|Z|T|H):\s*-?\d+(?:\.\d*)?/);
                            if (fullMatch) {
                                const wideMatch = s.match(/[XYZTH]:\s*.*?(?=\s[XYZTH]:|$)/g);
                                const raw = wideMatch ? wideMatch.join(' ') : fullMatch[0];
                                console.log(`[SMART-HIT] Found candidate: "${raw}"`);
                                candidates.push({ raw: raw, len: raw.length, crcOk: false, method: `smart_sym_off${offset}` });
                            }
                        }
                    }
                    validRun = 0; // Reset for next run
                }
            }
        }
    }


    // === PHASE 2: CRC-based frame scanning (no start symbol needed) ===
    if (candidates.length === 0) {
        const crcResults = crcFrameScan(bits);
        if (crcResults.length > 0) return crcResults;

        // Also try inverted bits
        if (candidates.length === 0) {
            const invBits = bits.map(b => b === 1 ? 0 : 1);
            const crcInvResults = crcFrameScan(invBits);
            for (const r of crcInvResults) candidates.push({ ...r, method: r.method + '_inv' });
        }
    }

    // === PHASE 3: Raw 8-bit text scan (no 4-to-6 encoding) ===
    if (candidates.length === 0) {
        const rawResults = rawTextScan(bits);
        for (const r of rawResults) candidates.push(r);

        if (candidates.length === 0) {
            const invBits = bits.map(b => b === 1 ? 0 : 1);
            const rawInvResults = rawTextScan(invBits);
            for (const r of rawInvResults) candidates.push({ ...r, method: r.method + '_inv' });
        }
    }

    // === PHASE 4: Brute-force 4-to-6 text scan (strict 5-field validation) ===
    if (candidates.length === 0) {
        const bruteResults = bruteForceScn(bits);
        for (const r of bruteResults) candidates.push({ ...r, method: 'brute_4to6' });

        if (candidates.length === 0) {
            const invBits = bits.map(b => b === 1 ? 0 : 1);
            const bruteInvResults = bruteForceScn(invBits);
            for (const r of bruteInvResults) candidates.push({ ...r, method: 'brute_4to6_inv' });
        }
    }

    return candidates;
}

/**
 * Parse telemetry string into structured data.
 * Requires ALL 5 fields (X, Y, Z, T, H) for valid=true.
 */
function parseTelemetry(msg, rssi) {
    const pkt = {
        timestamp: Date.now(),
        x: 0, y: 0, z: 0,
        temp: 0, humidity: 0,
        rssi: rssi,
        valid: false,
        raw: msg
    };

    try {
        const xM = msg.match(/X:\s*(-?\d+)/);
        const yM = msg.match(/Y:\s*(-?\d+)/);
        const zM = msg.match(/Z:\s*(-?\d+)/);
        const tM = msg.match(/T:\s*(-?\d+\.?\d*)/);
        const hM = msg.match(/H:\s*(-?\d+\.?\d*)/);

        if (xM) pkt.x = parseInt(xM[1], 10);
        if (yM) pkt.y = parseInt(yM[1], 10);
        if (zM) pkt.z = parseInt(zM[1], 10);
        if (tM) pkt.temp = parseFloat(tM[1]);
        if (hM) pkt.humidity = parseFloat(hM[1]);

        if (xM && yM && zM && tM && hM) {
            pkt.valid = true;
        }
    } catch (_) { }

    return pkt;
}

// ═══════════════════════════════════════════════════════════════
// STATE & SERVER
// ═══════════════════════════════════════════════════════════════
let curFreq = DEFAULT_FREQ, curGain = DEFAULT_GAIN, curRate = DEFAULT_RATE;
let rtlProc = null, capturing = false;
let prevPsd = null;
let lastSpectrum = 0, lastIQ = 0, lastDecode = 0;
let diagCounter = 0;
let decodeBuf = Buffer.alloc(0), iqRemainder = Buffer.alloc(0);

const wss = new WebSocket.Server({ port: WS_PORT });
console.log('Server started on port', WS_PORT);
const clients = new Set();
wss.on('connection', ws => {
    clients.add(ws);
    send(ws, { type: 'status', capturing, frequency: curFreq, sampleRate: curRate, gain: curGain, rtlAvailable: !!findRtlSdr() });
    ws.on('message', m => { try { handleCmd(JSON.parse(m)); } catch (_) { } });
    ws.on('close', () => clients.delete(ws));
    ws.on('error', () => clients.delete(ws));
});
function send(ws, msg) { if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg)); }
function broadcast(msg) { const j = JSON.stringify(msg); for (const c of clients) if (c.readyState === WebSocket.OPEN) c.send(j); }
function broadcastBin(buf) { for (const c of clients) if (c.readyState === WebSocket.OPEN) c.send(buf); }

function handleCmd(cmd) {
    if (cmd.action === 'start') startCapture(cmd.frequency || curFreq, cmd.gain ?? curGain, cmd.sampleRate || curRate);
    else if (cmd.action === 'stop') stopCapture();
    else if (cmd.action === 'setFreq') { curFreq = cmd.frequency; restartCapture(); }
    else if (cmd.action === 'setGain') { curGain = cmd.gain; restartCapture(); }
    else if (cmd.action === 'testPacket') sendTestPacket();
}

/**
 * Generate and decode a test packet through the FULL RH_ASK pipeline.
 * This proves the start-symbol + CRC decoder works correctly.
 */
function sendTestPacket() {
    const x = Math.floor(Math.random() * 1000 - 500);
    const y = Math.floor(Math.random() * 1000 - 500);
    const z = Math.floor(Math.random() * 1000 - 500);
    const t = (Math.floor(Math.random() * 150 + 200) / 10).toFixed(2);
    const h = (Math.floor(Math.random() * 600 + 300) / 10).toFixed(2);
    const msg = `X:${x} Y:${y} Z:${z} T:${t} H:${h}`;

    // Build a full RH_ASK frame
    const payloadBytes = [];
    for (let i = 0; i < msg.length; i++) payloadBytes.push(msg.charCodeAt(i));
    const totalLen = 1 + 4 + payloadBytes.length + 2;
    const frameBytes = [totalLen, 0xFF, 0xFF, 0x00, 0x00, ...payloadBytes];
    const crc = computeCRC(frameBytes, 0, frameBytes.length);
    frameBytes.push(crc & 0xFF);
    frameBytes.push((crc >> 8) & 0xFF);

    // Encode to bits: preamble + start symbol + 4-to-6 data
    const bits = [];
    for (let i = 0; i < 36; i++) bits.push(i % 2); // Preamble
    for (const b of START_SYMBOL) bits.push(b);     // Start symbol
    for (const byte of frameBytes) {
        const hi = S46[(byte >> 4) & 0x0F];
        const lo = S46[byte & 0x0F];
        for (let j = 5; j >= 0; j--) bits.push((hi >> j) & 1);
        for (let j = 5; j >= 0; j--) bits.push((lo >> j) & 1);
    }

    // Run decoder on the synthetic bit stream
    const candidates = scanForPackets(bits);
    if (candidates.length > 0) {
        candidates.sort((a, b) => {
            if (a.crcOk !== b.crcOk) return a.crcOk ? -1 : 1;
            return b.len - a.len;
        });
        const best = candidates[0];
        const pkt = parseTelemetry(best.raw, -50);
        const crcTag = best.crcOk ? 'CRC✓' : 'CRC✗';
        console.log(`[TEST] ${crcTag} [${best.method}] "${best.raw}" → X:${pkt.x} Y:${pkt.y} Z:${pkt.z} T:${pkt.temp} H:${pkt.humidity}`);
        broadcast({ type: 'packet', data: pkt });
    } else {
        console.log(`[TEST] FAILED: No packet decoded from synthetic frame for "${msg}"`);
        broadcast({ type: 'packet', data: { timestamp: Date.now(), x: 0, y: 0, z: 0, temp: 0, humidity: 0, rssi: -50, valid: false, raw: msg } });
    }
}

function restartCapture() { if (capturing) { stopCapture(); setTimeout(() => startCapture(curFreq, curGain, curRate), 300); } }

function startCapture(freq, gain, rate) {
    if (capturing) stopCapture();
    const exe = findRtlSdr();
    if (!exe) { broadcast({ type: 'error', message: 'rtl_sdr.exe not found' }); return; }
    curFreq = freq; curGain = gain; curRate = rate;
    prevPsd = null; decodeBuf = Buffer.alloc(0); iqRemainder = Buffer.alloc(0); diagCounter = 0;
    const args = ['-f', freq, '-s', rate, '-g', gain, '-'];
    console.log(`[RTL] Start: ${args.join(' ')}`);
    rtlProc = spawn(exe, args, { stdio: ['ignore', 'pipe', 'pipe'] });
    capturing = true;
    broadcast({ type: 'status', capturing: true, frequency: freq, sampleRate: rate, gain, rtlAvailable: true });

    rtlProc.stdout.on('data', processChunk);
    rtlProc.stderr.on('data', d => { const m = d.toString().trim(); if (m) console.log(`[RTL] ${m}`); });
    rtlProc.on('close', () => { capturing = false; rtlProc = null; broadcast({ type: 'status', capturing: false }); broadcast({ type: 'clear' }); });
    rtlProc.on('error', e => { capturing = false; rtlProc = null; broadcast({ type: 'error', message: e.message }); });
}
function stopCapture() { if (rtlProc) { try { rtlProc.kill() } catch (_) { } rtlProc = null; } capturing = false; broadcast({ type: 'status', capturing: false }); broadcast({ type: 'clear' }); }

function processChunk(chunk) {
    iqRemainder = Buffer.concat([iqRemainder, chunk]);
    decodeBuf = Buffer.concat([decodeBuf, chunk]);
    const now = Date.now();

    while (iqRemainder.length >= FFT_SIZE * 2) {
        const b = iqRemainder.slice(0, FFT_SIZE * 2); iqRemainder = iqRemainder.slice(FFT_SIZE * 2);
        if (now - lastSpectrum >= SPECTRUM_INTERVAL) {
            lastSpectrum = now; const p = computePSD(b, 0, FFT_SIZE);
            if (prevPsd) for (let i = 0; i < p.length; i++) p[i] = 0.35 * p[i] + 0.65 * prevPsd[i];
            prevPsd = Float32Array.from(p);
            const bin = Buffer.alloc(1 + p.byteLength); bin[0] = 0x01; Buffer.from(p.buffer).copy(bin, 1); broadcastBin(bin);
        }
        if (now - lastIQ >= IQ_INTERVAL) {
            lastIQ = now;
            const pts = []; const step = Math.max(2, Math.floor(b.length / 100));
            for (let i = 0; i < b.length - 1 && pts.length < 64; i += step) pts.push({ i: +((b[i] - 127.5) / 127.5).toFixed(3), q: +((b[i + 1] - 127.5) / 127.5).toFixed(3) });
            broadcast({ type: 'iq', points: pts });
        }
    }

    const decodeBytes = Math.floor(curRate * 2 * DECODE_WINDOW_MS / 1000);
    if (decodeBuf.length >= decodeBytes && now - lastDecode >= 400) {
        lastDecode = now;
        runDecoder(decodeBuf);
        const keep = Math.floor(decodeBuf.length * 0.4);
        decodeBuf = decodeBuf.slice(decodeBuf.length - keep);
    }
}

// ═══════════════════════════════════════════════════════════════
// DYNAMIC SLICER
// ═══════════════════════════════════════════════════════════════
function dynamicSlicer(env, nominalSPB) {
    const pulses = [];
    const N = env.length;

    // Calculate Moving Average (MA) as a dynamic threshold
    // Window size: ~8 bit periods to capture the average DC level of the signal + noise
    const winSize = Math.max(10, Math.floor(nominalSPB * 8));
    const ma = new Float64Array(N);

    let sum = 0;
    for (let i = 0; i < N; i++) {
        sum += env[i];
        if (i >= winSize) sum -= env[i - winSize];
        ma[i] = sum / Math.min(i + 1, winSize);
    }

    const sliceWin = Math.max(5, Math.floor(nominalSPB * 4));
    let state = 0;
    let count = 0;
    let threshold = 0;

    for (let i = 0; i < N; i++) {
        // Threshold = MA[i]
        threshold = ma[i];

        // Hysteresis
        const highThr = threshold * 1.1; // +10%
        const lowThr = threshold * 0.9;  // -10%

        if (state === 0 && env[i] > highThr) {
            if (count > 0) pulses.push({ level: 0, samples: count });
            state = 1; count = 1;
        } else if (state === 1 && env[i] < lowThr) {
            if (count > 0) pulses.push({ level: 1, samples: count });
            state = 0; count = 1;
        } else {
            count++;
        }
    }
    if (count > 0) pulses.push({ level: state, samples: count });
    return pulses;
}

function runDecoder(buf) {
    const nSamples = Math.floor(buf.length / 2);
    const nominalSPB = curRate / TARGET_BITRATE;

    // STEP 1: Remove DC offset (0 Hz spike) from raw I/Q
    // This is critical when using wide LPFs to ensure the DC spike doesn't domimate
    const { iOut: cleanI, qOut: cleanQ } = removeDC(buf, 0, nSamples);

    // PATH A: LPF'd IQ → envelope
    // Wide 120kHz LPF to capture signals even if 60-80kHz off-center
    // We filter the CLEAN (DC-removed) I/Q data
    const { filtI, filtQ } = lowPassFilterArrays(cleanI, cleanQ, curRate, 15000);
    const envA = smoothEnv(envelopeIQ(filtI, filtQ), Math.max(1, Math.floor(nominalSPB / 4)));

    // PATH B: Raw IQ envelope (Wideband, DC-removed)
    const envB = smoothEnv(envelopeIQ(cleanI, cleanQ), Math.max(2, Math.floor(nominalSPB / 3)));

    const allCandidates = [];
    let bestBits = [];
    let bestSNR = 0;
    let bestThr = null;

    // Use Dynamic Slicer on both paths
    for (const [label, env] of [['lpf', envA], ['raw', envB]]) {
        // console.error(`[LOOP] Label=${label} EnvLen=${env.length}`); 

        // Try DYNAMIC SLICER
        const pulses = dynamicSlicer(env, nominalSPB);

        // Calculate a "pseudo-SNR" for metrics based on global variability
        const thr = adaptiveThreshold(env); // Just for logging/display stats
        const snr = thr.range > 0 ? 20 * Math.log10(thr.max / Math.max(thr.min + 0.001, 0.001)) : 0;

        // console.error(`[SNR] Label=${label} SNR=${snr.toFixed(1)} Pulses=${pulses.length}`);

        if (snr > bestSNR) { bestSNR = snr; bestThr = thr; }

        if (label === 'lpf' && snr > 10) {
            console.log(`[DEBUG] SNR=${snr.toFixed(1)} Pulses=${pulses.length} Range=${thr.range.toFixed(4)}`);
            if (pulses.length > 0) {
                const bits = pulsesToBits(pulses, nominalSPB);
                // console.log(`[BITS] ${bits.slice(0, 100).join('')}`);
            }
        }
        const adaptSPB = estimateSPB(pulses, nominalSPB);

        // Expanded SPB search for clock drift
        const spbCandidates = [nominalSPB, adaptSPB];
        for (let f = 0.90; f <= 1.10; f += 0.01) spbCandidates.push(nominalSPB * f);

        for (const spb of spbCandidates) {
            const bits = pulsesToBits(pulses, spb);
            if (bits.length < 50) continue;

            // DEBUG: Print bits if SNR is good (relaxed condition)
            if (snr > 8 && label === 'lpf') {
                const head = bits.slice(0, 100).join('');
                console.log(`[DEBUG] SNR=${snr.toFixed(1)} SPB=${spb.toFixed(1)} (nom=${nominalSPB}) Bits=${head}...`);
            }

            // v25: Scan BOTH normal and Inverted streams
            const bitsInv = bits.map(b => 1 - b);

            for (const [bStream, pol] of [[bits, 'NORM'], [bitsInv, 'INV']]) {
                const packets = scanForPackets(bStream);
                for (const p of packets) {
                    allCandidates.push({ ...p, spb, bitrate: Math.round(curRate / spb), path: `${label}/${pol}` });
                }
                if (bStream.length > bestBits.length) bestBits = bStream; // Keep longest for diag
            }
        }
    }

    const thr = bestThr || adaptiveThreshold(envA);
    const rssi = 20 * Math.log10(Math.max((thr.max + thr.min) / 2, 0.001) / 127.5) - 40;
    const snr = bestSNR;

    if (allCandidates.length > 0) {
        allCandidates.sort((a, b) => {
            if (a.crcOk !== b.crcOk) return a.crcOk ? -1 : 1;
            return b.len - a.len;
        });
        const best = allCandidates[0];
        const now = Date.now();
        if (best.raw === lastDecodedRaw && (now - lastDecodedTime) < DEDUP_WINDOW_MS) {
            broadcast({ type: 'metrics', snr: +snr.toFixed(1), rssi: +rssi.toFixed(0), peak: +thr.max.toFixed(1), bits: bestBits.length, bitrate: best.bitrate });
            return;
        }
        lastDecodedRaw = best.raw;
        lastDecodedTime = now;
        const pkt = parseTelemetry(best.raw, rssi);
        const crcTag = best.crcOk ? 'CRC✓' : 'CRC✗';
        console.log(`[DECODE] ${crcTag} [${best.method}/${best.path}] ${best.bitrate}bps "${best.raw}"`);
        console.log(`  → X:${pkt.x} Y:${pkt.y} Z:${pkt.z} T:${pkt.temp} H:${pkt.humidity} valid:${pkt.valid}`);
        broadcast({ type: 'packet', data: pkt });
        broadcast({ type: 'metrics', snr: +snr.toFixed(1), rssi: +rssi.toFixed(0), peak: +thr.max.toFixed(1), bits: bestBits.length, bitrate: best.bitrate });
        return;
    }

    // No packet — diagnostic
    diagCounter++;
    if (diagCounter % 5 === 0) {
        const thrA = adaptiveThreshold(envA);
        const snrA = thrA.range > 0 ? 20 * Math.log10(thrA.max / Math.max(thrA.min + 0.001, 0.001)) : 0;
        console.log(`[DIAG-v28] SNR=${snrA.toFixed(1)}dB (Scanning...)`);
    }
    broadcast({ type: 'metrics', snr: +snr.toFixed(1), rssi: +rssi.toFixed(0), peak: +thr.max.toFixed(1), bits: bestBits.length, bitrate: TARGET_BITRATE });
}

console.log(`\n\n═══════════════════════════════════════════════════════════════`);
console.log(`\n\n═══════════════════════════════════════════════════════════════`);
console.log(`   SDR BRIDGE v28 - DEEP INSPECTION`);
console.log(`═══════════════════════════════════════════════════════════════\n\n`);
process.on('SIGINT', () => { stopCapture(); wss.close(); process.exit(0); });
process.on('SIGTERM', () => { stopCapture(); wss.close(); process.exit(0); });
startCapture(DEFAULT_FREQ, DEFAULT_GAIN, DEFAULT_RATE);
