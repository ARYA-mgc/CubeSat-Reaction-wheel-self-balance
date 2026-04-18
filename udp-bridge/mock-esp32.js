/**
 * Mock ESP32 Simulator
 * Sends rotating quaternion data via UDP to test the bridge server
 * 
 * Usage: node udp-bridge/mock-esp32.js
 */

const dgram = require('dgram');

// Configuration
const UDP_PORT = 4210;
const TARGET_HOST = '127.0.0.1';
const UPDATE_RATE = 60; // Hz

const client = dgram.createSocket('udp4');

let angle = 0;
const rotationSpeed = 0.5; // radians per second

console.log('');
console.log('╔═══════════════════════════════════════════════════════════════╗');
console.log('║          Mock ESP32 - Quaternion Sender                       ║');
console.log('╠═══════════════════════════════════════════════════════════════╣');
console.log(`║  Sending to ${TARGET_HOST}:${UDP_PORT}                                 ║`);
console.log(`║  Update rate: ${UPDATE_RATE} Hz                                        ║`);
console.log('║  Simulating rotation around Y-axis                            ║');
console.log('╠═══════════════════════════════════════════════════════════════╣');
console.log('║  Press Ctrl+C to stop                                         ║');
console.log('╚═══════════════════════════════════════════════════════════════╝');
console.log('');

function sendQuaternion() {
    // Create quaternion for rotation around Y-axis
    // q = [sin(θ/2) * axis, cos(θ/2)]
    // For Y-axis rotation: q = [0, sin(θ/2), 0, cos(θ/2)]
    const halfAngle = angle / 2;

    // Add some wobble on other axes for more interesting motion
    const wobble = Math.sin(angle * 2) * 0.1;

    const i = Math.sin(wobble);          // X component
    const j = Math.sin(halfAngle);       // Y component (main rotation)
    const k = Math.cos(wobble) * 0.05;   // Z component
    const real = Math.cos(halfAngle);    // W component

    // Normalize the quaternion
    const mag = Math.sqrt(i * i + j * j + k * k + real * real);
    const qi = i / mag;
    const qj = j / mag;
    const qk = k / mag;
    const qreal = real / mag;

    // Format: "i, j, k, real"
    const message = `${qi.toFixed(4)}, ${qj.toFixed(4)}, ${qk.toFixed(4)}, ${qreal.toFixed(4)}`;

    client.send(message, UDP_PORT, TARGET_HOST, (err) => {
        if (err) {
            console.error('Send error:', err);
        }
    });

    // Update angle for next frame
    angle += rotationSpeed / UPDATE_RATE;
    if (angle > Math.PI * 2) {
        angle -= Math.PI * 2;
    }

    // Log occasionally
    if (Math.random() < 0.02) {
        console.log(`[Sending] i=${qi.toFixed(3)}, j=${qj.toFixed(3)}, k=${qk.toFixed(3)}, w=${qreal.toFixed(3)}`);
    }
}

// Start sending at specified rate
const interval = setInterval(sendQuaternion, 1000 / UPDATE_RATE);

// Graceful shutdown
process.on('SIGINT', () => {
    console.log('\n[Mock ESP32] Stopping...');
    clearInterval(interval);
    client.close();
    process.exit(0);
});
