/**
 * test_decoder.js — v3
 * Tests the RH_ASK protocol-aware decoder with:
 *   - Full RH_ASK frame encoding (preamble + start symbol + 4-to-6 + CRC)
 *   - Start symbol detection
 *   - CRC-CCITT verification
 *   - Round-trip encoding → decoding for positive, negative, decimal, and zero values
 */

// ─── RadioHead 4-to-6 Encoding Table ─────────────────────────────────────
const S46 = [0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34];
const S64 = new Array(64).fill(-1);
for (let i = 0; i < 16; i++) S64[S46[i]] = i;

// RH_ASK start symbol: 0xB38
const START_SYMBOL = [1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0];

// ─── CRC-CCITT (matches RadioHead exactly) ──────────────────────────────

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

// ─── RH_ASK Frame Encoder (simulates Arduino + RadioHead) ───────────────

function encodeRhAskFrame(payloadStr) {
    // Build frame bytes: [len] [to=0xFF] [from=0xFF] [id=0x00] [flags=0x00] [payload...] [crc_lo] [crc_hi]
    const payloadBytes = [];
    for (let i = 0; i < payloadStr.length; i++) payloadBytes.push(payloadStr.charCodeAt(i));

    const headerLen = 4; // to, from, id, flags
    const totalLen = 1 + headerLen + payloadBytes.length + 2; // len + header + payload + CRC
    const frameBytes = [totalLen, 0xFF, 0xFF, 0x00, 0x00, ...payloadBytes];

    // Compute CRC over bytes 0..frameBytes.length-1
    const crc = computeCRC(frameBytes, 0, frameBytes.length);
    frameBytes.push(crc & 0xFF);
    frameBytes.push((crc >> 8) & 0xFF);

    // Encode to bits: preamble + start symbol + 4-to-6 encoded data
    const bits = [];

    // Preamble: 36 bits of alternating 1/0 (training for AGC)
    for (let i = 0; i < 36; i++) bits.push(i % 2);

    // Start symbol: 0xB38
    for (const b of START_SYMBOL) bits.push(b);

    // 4-to-6 encode each byte
    for (const byte of frameBytes) {
        const hi = (byte >> 4) & 0x0F;
        const lo = byte & 0x0F;
        const sym1 = S46[hi];
        const sym2 = S46[lo];
        for (let j = 5; j >= 0; j--) bits.push((sym1 >> j) & 1);
        for (let j = 5; j >= 0; j--) bits.push((sym2 >> j) & 1);
    }

    // Trailing padding
    for (let i = 0; i < 24; i++) bits.push(0);

    return bits;
}

// ─── Decoder (copied from server.cjs v12) ────────────────────────────────

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
            break;
        }
    }
    return bytes;
}

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
        if (errors <= maxErrors) positions.push(i + pattern.length);
    }
    return positions;
}

function tryDecodeFrame(bits, dataStart) {
    const bytes = decode4to6(bits, dataStart, 80);
    if (bytes.length < 7) return null;

    const frameLen = bytes[0];
    if (frameLen < 7 || frameLen > 70) return null;
    if (bytes.length < frameLen) return null;

    const crcComputed = computeCRC(bytes, 0, frameLen - 2);
    const crcReceived = (bytes[frameLen - 2]) | (bytes[frameLen - 1] << 8);
    const crcOk = (crcComputed === crcReceived);

    const payloadBytes = bytes.slice(5, frameLen - 2);
    let payload = '';
    for (const b of payloadBytes) {
        if (b >= 32 && b <= 126) payload += String.fromCharCode(b);
    }

    if (payload.length < 3) return null;
    return { raw: payload, len: payload.length, crcOk, frameLen };
}

function parseTelemetry(msg) {
    const pkt = { x: 0, y: 0, z: 0, temp: 0, humidity: 0, valid: false, raw: msg };
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
    if (xM && yM && zM && tM && hM) pkt.valid = true;
    return pkt;
}

// ─── Test Harness ────────────────────────────────────────────────────────

let pass = 0, fail = 0;

function runTest(name, input, expected) {
    console.log(`\n--- ${name} ---`);
    console.log(`  Input:    "${input}"`);

    // Encode as a full RH_ASK frame
    const bits = encodeRhAskFrame(input);
    console.log(`  Bits:     ${bits.length} (preamble=36, start=12, data=${(input.length + 7) * 12})`);

    // Find start symbol
    const startPositions = findStartSymbols(bits, START_SYMBOL, 0);
    if (startPositions.length === 0) {
        console.log(`  ✗ FAILED: Start symbol not found`);
        fail++;
        return;
    }
    console.log(`  Start:    found at bit ${startPositions[0] - 12}, decode begins at ${startPositions[0]}`);

    // Decode frame
    const frame = tryDecodeFrame(bits, startPositions[0]);
    if (!frame) {
        console.log(`  ✗ FAILED: Frame decode returned null`);
        fail++;
        return;
    }

    console.log(`  CRC:      ${frame.crcOk ? '✓ VALID' : '✗ INVALID'}`);
    console.log(`  Payload:  "${frame.raw}"`);

    if (!frame.crcOk) {
        console.log(`  ✗ FAILED: CRC mismatch`);
        fail++;
        return;
    }

    // Verify payload matches input
    if (frame.raw !== input) {
        console.log(`  ✗ FAILED: Payload mismatch`);
        console.log(`    Expected: "${input}"`);
        console.log(`    Got:      "${frame.raw}"`);
        fail++;
        return;
    }

    // Parse telemetry values
    const pkt = parseTelemetry(frame.raw);
    console.log(`  Parsed:   X:${pkt.x} Y:${pkt.y} Z:${pkt.z} T:${pkt.temp} H:${pkt.humidity} valid:${pkt.valid}`);

    let ok = true;
    for (const key of Object.keys(expected)) {
        if (pkt[key] !== expected[key]) {
            console.log(`  ✗ MISMATCH: ${key} expected=${expected[key]} got=${pkt[key]}`);
            ok = false;
        }
    }

    if (ok) {
        console.log(`  ✓ PASS — CRC valid, payload exact, all values correct`);
        pass++;
    } else {
        fail++;
    }
}

// ─── Test Cases ──────────────────────────────────────────────────────────

runTest("All Positive Values",
    "X:307 Y:251 Z:427 T:20.80 H:43.00",
    { x: 307, y: 251, z: 427, temp: 20.8, humidity: 43, valid: true }
);

runTest("Negative X, Y, Z",
    "X:-307 Y:-251 Z:-427 T:20.80 H:43.00",
    { x: -307, y: -251, z: -427, temp: 20.8, humidity: 43, valid: true }
);

runTest("Mixed Positive and Negative",
    "X:100 Y:-200 Z:300 T:-5.50 H:88.20",
    { x: 100, y: -200, z: 300, temp: -5.5, humidity: 88.2, valid: true }
);

runTest("Zero Values",
    "X:0 Y:0 Z:0 T:0.00 H:0.00",
    { x: 0, y: 0, z: 0, temp: 0, humidity: 0, valid: true }
);

runTest("Large Negative Values",
    "X:-999 Y:-500 Z:-1 T:-40.00 H:100.00",
    { x: -999, y: -500, z: -1, temp: -40, humidity: 100, valid: true }
);

runTest("Decimal Temperature and Humidity",
    "X:42 Y:-17 Z:512 T:36.75 H:62.30",
    { x: 42, y: -17, z: 512, temp: 36.75, humidity: 62.3, valid: true }
);

runTest("Exact User Packet (from RF capture)",
    "X:307 Y:-251 Z:-427 T:20.80 H:43.00",
    { x: 307, y: -251, z: -427, temp: 20.8, humidity: 43, valid: true }
);

runTest("Second User Packet",
    "X:-228 Y:44 Z:378 T:22.30 H:40.90",
    { x: -228, y: 44, z: 378, temp: 22.3, humidity: 40.9, valid: true }
);

runTest("Third User Packet (large negatives)",
    "X:-60 Y:-335 Z:-8 T:29.20 H:68.70",
    { x: -60, y: -335, z: -8, temp: 29.2, humidity: 68.7, valid: true }
);

// ─── Summary ─────────────────────────────────────────────────────────────

console.log(`\n${'═'.repeat(60)}`);
console.log(`  RESULTS: ${pass} passed, ${fail} failed out of ${pass + fail} tests`);
console.log(`${'═'.repeat(60)}`);

if (fail > 0) process.exit(1);
