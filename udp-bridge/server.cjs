/**
 * UDP-to-WebSocket Bridge Server
 * Receives quaternion data from ESP32 via UDP and broadcasts to web clients via WebSocket
 * 
 * Usage: node udp-bridge/server.js
 */

const dgram = require('dgram');
const WebSocket = require('ws');

// Configuration
const UDP_PORT = 4210;
const WS_PORT = 8765;

// Create UDP server
const udpServer = dgram.createSocket('udp4');

// Create WebSocket server
const wss = new WebSocket.Server({ port: WS_PORT });

// Track connected WebSocket clients
const clients = new Set();

// Track last seen ESP32 address for remote control
let lastEsp32Addr = null;

// WebSocket connection handling
wss.on('connection', (ws) => {
  clients.add(ws);
  console.log(`[WS] Client connected. Total clients: ${clients.size}`);

  ws.on('message', (msg) => {
    try {
      const data = JSON.parse(msg);
      // Relay command types to ESP32
      const commandTypes = ['control', 'rotate_rel', 'calibrate', 'motor_toggle'];
      if (commandTypes.includes(data.type) && lastEsp32Addr) {
        // Send the full JSON so ESP32 can distinguish types
        const cmdBuffer = Buffer.from(JSON.stringify(data));
        udpServer.send(cmdBuffer, UDP_PORT, lastEsp32Addr, (err) => {
          if (err) console.error('[UDP] Relay error:', err);
          else console.log(`[UDP] Relayed ${data.type} to ${lastEsp32Addr}`);
        });
      }
    } catch (e) {
      // Ignore non-JSON messages
    }
  });

  ws.on('close', () => {
    clients.delete(ws);
    console.log(`[WS] Client disconnected. Total clients: ${clients.size}`);
  });

  ws.on('error', (err) => {
    console.error('[WS] Client error:', err.message);
    clients.delete(ws);
  });
});

// UDP message handling
udpServer.on('message', (msg, rinfo) => {
  const dataStr = msg.toString().trim();

  try {
    // Attempt to parse JSON
    // Expected format: {"quat":{"i":...,"j":...,"k":...,"real":...},"env":{"temp":...,"hum":...},"motor":[...]}
    let jsonData;

    // Support legacy format (csv) just in case, or assume JSON
    if (dataStr.startsWith('{')) {
      jsonData = JSON.parse(dataStr);
    } else {
      // Parse legacy comma-separated quaternion: "i, j, k, real"
      const parts = dataStr.split(',').map(s => parseFloat(s.trim()));
      if (parts.length === 4 && parts.every(n => !isNaN(n))) {
        jsonData = {
          quat: { i: parts[0], j: parts[1], k: parts[2], real: parts[3] },
          env: { temp: 0, hum: 0, alt: 0 },
          motor: [0, 0, 0]
        };
      }
    }

    if (jsonData && jsonData.quat) {
      lastEsp32Addr = rinfo.address;
      // Standardize for Frontend
      const packet = {
        type: 'telemetry',
        payload: {
          quaternion: {
            i: jsonData.quat.i,
            j: jsonData.quat.j,
            k: jsonData.quat.k,
            real: jsonData.quat.real || jsonData.quat.w // support both naming
          },
          environment: jsonData.env,
          motor: jsonData.motor,
          timestamp: Date.now()
        }
      };

      // Broadcast to all WebSocket clients
      const jsonString = JSON.stringify(packet);
      let sentCount = 0;
      clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(jsonString);
          sentCount++;
        }
      });

      // Log occasionally
      if (Math.random() < 0.05) {
        console.log(`[UDP->WS] Telemetry relayed to ${sentCount} clients`);
      }
    }
  } catch (e) {
    console.warn(`[UDP] Failed to parse data from ${rinfo.address}: "${dataStr}"`, e.message);
  }
});

udpServer.on('listening', () => {
  const address = udpServer.address();
  console.log('');
  console.log('╔═══════════════════════════════════════════════════════════════╗');
  console.log('║          CubeDynamics - IMU Bridge Server                     ║');
  console.log('╠═══════════════════════════════════════════════════════════════╣');
  console.log(`║  UDP Server listening on port ${address.port}                         ║`);
  console.log(`║  WebSocket Server running on port ${WS_PORT}                         ║`);
  console.log('╠═══════════════════════════════════════════════════════════════╣');
  console.log('║  Waiting for ESP32 data...                                    ║');
  console.log('║  Format: "i, j, k, real" (quaternion components)              ║');
  console.log('╚═══════════════════════════════════════════════════════════════╝');
  console.log('');
});

udpServer.on('error', (err) => {
  console.error('[UDP] Server error:', err);
  udpServer.close();
});

// Start UDP server
udpServer.bind(UDP_PORT);

// Graceful shutdown
process.on('SIGINT', () => {
  console.log('\n[Server] Shutting down...');
  udpServer.close();
  wss.close();
  process.exit(0);
});
