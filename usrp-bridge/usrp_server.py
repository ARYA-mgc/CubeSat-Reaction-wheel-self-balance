import asyncio
import json
import numpy as np
import websockets
import time
import struct
import math
import sys
import os
import socket

# --- GNURadio 3.10 Path Injection ---
GR_ROOT = r"C:\GNURadio-3.10"
if os.path.exists(GR_ROOT):
    gr_packages = [
        os.path.join(GR_ROOT, "lib", "site-packages"),
        os.path.join(GR_ROOT, "python", "Lib"),
        os.path.join(GR_ROOT, "bin"),
    ]
    for p in gr_packages:
        if p not in sys.path: sys.path.insert(0, p)
    
    if os.name == 'nt':
        # Add DLL directory for UHD and GR dependencies
        os.add_dll_directory(os.path.join(GR_ROOT, "bin"))
        # Update PATH for child processes
        os.environ['PATH'] = os.path.join(GR_ROOT, "bin") + ";" + os.environ.get('PATH', '')

try:
    from gnuradio import uhd, gr, blocks
    try:
        from gnuradio import network
        UDP_SINK_MOD = network
    except ImportError:
        UDP_SINK_MOD = blocks
    USRP_AVAILABLE = True
    print("[BOOT] SUCCESS: Loaded GNURadio/UHD modules")
except Exception as e:
    print(f"[BOOT] CRITICAL: GNURadio load failed: {e}")
    USRP_AVAILABLE = False
    UDP_SINK_MOD = None

# --- Config ---
WS_PORT = 8766
UDP_PORT = 9999
DEFAULT_FREQ = 2.4e9
DEFAULT_RATE = 1e6
DEFAULT_GAIN = 40

class USRPFlowgraph(gr.top_block):
    def __init__(self, freq, rate, gain, udp_port):
        gr.top_block.__init__(self, "USRP Bridge Flowgraph")
        
        # 1. USRP Source
        st_args = uhd.stream_args("fc32", "sc16")
        self.src = uhd.usrp_source("type=b200", st_args)
        self.src.set_samp_rate(rate)
        self.src.set_center_freq(uhd.tune_request(freq), 0)
        self.src.set_gain(gain, 0)
        
        # 2. UDP Sink (Internal bridge)
        # We use a socket sink to stream samples to our local Python listener
        if hasattr(UDP_SINK_MOD, 'udp_sink'):
            try:
                # GR 3.10 (network module) often expects 7 args:
                # itemsize, veclen, host, port, header_type, payloadsize, send_eof
                self.snk = UDP_SINK_MOD.udp_sink(gr.sizeof_gr_complex, 1, "127.0.0.1", udp_port, 0, 1472, True)
            except TypeError:
                # Fallback for older versions or different builds (5 args)
                self.snk = UDP_SINK_MOD.udp_sink(gr.sizeof_gr_complex, "127.0.0.1", udp_port, 1472, True)
        
        self.connect(self.src, self.snk)

class USRPBridge:
    def __init__(self):
        self.running = False
        self.tb = None
        self.clients = set()
        self.freq = DEFAULT_FREQ
        self.rate = DEFAULT_RATE
        self.gain = DEFAULT_GAIN
        print("[BRIDGE] Instance created.")

    def start_hw(self):
        if not USRP_AVAILABLE: 
            asyncio.create_task(self.broadcast(json.dumps({"type": "error", "message": "GNURadio Environment Error"})))
            return
            
        # PROBE: Check if USRP is actually visible before attempting init
        try:
            found = uhd.find_devices()
            if not found:
                print("[USRP] CRITICAL: No USRP Hardware Found! Check USB connection.")
                asyncio.create_task(self.broadcast(json.dumps({
                    "type": "error", 
                    "message": "NO HARDWARE DETECTED: Is the USRP-2900 plugged in?",
                    "status": "disconnected"
                })))
                return
        except: pass

        try:
            print(f"[USRP] Initializing USRP-2900 (B200) at {self.freq/1e6} MHz...")
            if self.tb: self.stop_hw()
            self.tb = USRPFlowgraph(self.freq, self.rate, self.gain, UDP_PORT)
            self.tb.start()
            
            # Probe for metadata
            hw_info = {
                "type": "hw_info", 
                "model": "USRP-2900 (B200)", 
                "serial": "Connecting...",
                "mboard_name": "Ettus/NI", 
                "master_clock_rate": 0
            }
            try:
                usrp_info = self.tb.src.get_usrp_info()
                hw_info["model"] = usrp_info.get("mboard_id", "USRP B200")
                hw_info["serial"] = usrp_info.get("mboard_serial", "Unknown")
                if hasattr(self.tb.src, 'get_clock_rate'):
                    hw_info["master_clock_rate"] = self.tb.src.get_clock_rate(0)
            except: pass

            asyncio.create_task(self.broadcast(json.dumps(hw_info)))
            print(f"[USRP] Flowgraph started. Real HW Detected: {hw_info['model']} S/N: {hw_info['serial']}")
        except Exception as e:
            print(f"[USRP] Hardware Init Failed: {e}")
            asyncio.create_task(self.broadcast(json.dumps({"type": "error", "message": f"Hardware Init Error: {str(e)}"})))

    def stop_hw(self):
        if self.tb:
            print("[USRP] Stopping hardware streaming...")
            try:
                self.tb.stop()
                self.tb.wait()
            except: pass
            self.tb = None
        self.running = False

    async def broadcast(self, message):
        if not self.clients: return
        websockets.broadcast(self.clients, message)

    async def udp_listener(self):
        """ Listens to the GNURadio flowgraph and forwards to WebSockets """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", UDP_PORT))
        sock.setblocking(False)
        # Increase OS buffer size to prevent LIBUSB/UDP drops
        try: sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
        except: pass
        
        loop = asyncio.get_event_loop()
        print(f"[UDP] Bridge listening on port {UDP_PORT}")
        sample_buffer = np.array([], dtype=np.complex64)
        fft_size = 1024
        
        # Spectral Averaging
        avg_v = None
        alpha = 0.2 # Smoothing factor
        
        last_debug = time.time()
        self.last_rx_time = time.time()
        
        while True:
            # Watchdog: Clear UI if hardware stops (e.g. USB error)
            if self.running and (time.time() - self.last_rx_time > 1.5):
                await self.broadcast(json.dumps({
                    "type": "error", 
                    "message": "HARDWARE LOST: Check USB",
                    "status": "disconnected"
                }))
                await self.broadcast(json.dumps({"type": "clear"}))
                print("[USRP] Stream Lost! Waiting for hardware...")
                self.last_rx_time = time.time() # Reset to avoid spam

            if self.running and self.clients:
                try:
                    # Use wait_for to prevent hanging on recv if stream dies
                    data = await asyncio.wait_for(loop.sock_recv(sock, 16384), timeout=0.5)
                    if not data: continue
                    
                    self.last_rx_time = time.time()
                    new_samples = np.frombuffer(data, dtype=np.complex64)
                    sample_buffer = np.concatenate((sample_buffer, new_samples))
                    
                    # Log activity every 3 seconds for visual verification
                    if time.time() - last_debug > 3 and len(new_samples) > 0:
                        rssi_est = 10 * np.log10(np.mean(np.abs(new_samples)**2) + 1e-12)
                        print(f"[LIVE] RF Stream active. RSSI: {rssi_est:.1f} dBm | S/N: {self.tb.src.get_usrp_info().get('mboard_serial','?') if self.tb else '?'}")
                        last_debug = time.time()
                    
                    # Process FFT
                    while len(sample_buffer) >= fft_size:
                        samples = sample_buffer[:fft_size]
                        sample_buffer = sample_buffer[fft_size:]
                        
                        window = np.hanning(fft_size)
                        spectrum = np.fft.fftshift(np.fft.fft(samples * window)) / fft_size
                        psd = 10 * np.log10(np.abs(spectrum)**2 + 1e-14)
                        
                        # Averaging (Exponential Moving Average)
                        if avg_v is None or len(avg_v) != len(psd):
                            avg_v = psd.copy()
                        else:
                            avg_v = (1 - alpha) * avg_v + alpha * psd

                        # Binary broadcast
                        header = struct.pack('B', 0x01)
                        await self.broadcast(header + avg_v.astype(np.float32).tobytes())
                        
                        # Metrics
                        if np.random.random() < 0.15:
                            peak = float(np.max(avg_v))
                            noise = float(np.mean(avg_v))
                            await self.broadcast(json.dumps({
                                "type": "metrics", "rssi": noise, "snr": peak - noise, "peak": peak
                            }))
                except asyncio.TimeoutError:
                    continue
                except Exception as e:
                    await asyncio.sleep(0.01)
            else:
                if len(sample_buffer) > 0: sample_buffer = np.array([], dtype=np.complex64)
                await asyncio.sleep(0.1)

    async def handle_ws(self, websocket):
        self.clients.add(websocket)
        print(f"[WS] Client connected. Total: {len(self.clients)}")
        
        hw_present = False
        try: hw_present = len(uhd.find_devices()) > 0
        except: pass

        try:
            # Initial status broadcast
            await websocket.send(json.dumps({
                "type": "status", 
                "usrpAvailable": hw_present, 
                "capturing": self.running if self.tb else False,
                "frequency": self.freq, 
                "gain": self.gain, 
                "sampleRate": self.rate,
                "server_v": "3.3-perfecto"
            }))
            
            async for message in websocket:
                try:
                    data = json.loads(message)
                    action = data.get("action")
                    
                    if action == "start":
                        self.freq = data.get("frequency", self.freq)
                        self.gain = data.get("gain", self.gain)
                        self.running = True
                        self.start_hw()
                        if not self.tb:
                            self.running = False
                            await self.broadcast(json.dumps({
                                "type": "error", "message": "NO USRP DETECTED", "status": "disconnected"
                            }))
                        else:
                            await self.broadcast(json.dumps({"type": "status", "capturing": True}))
                    
                    elif action == "stop":
                        self.running = False
                        self.stop_hw()
                        await self.broadcast(json.dumps({"type": "status", "capturing": False}))
                        await self.broadcast(json.dumps({"type": "clear"}))

                    elif action == "setFreq":
                        target_f = data.get("frequency", self.freq)
                        if self.tb:
                            try:
                                self.tb.src.set_center_freq(uhd.tune_request(target_f), 0)
                                actual_f = self.tb.src.get_center_freq(0)
                                self.freq = actual_f
                                print(f"[HARDWARE] Tuned. Req: {target_f/1e6} MHz | Got: {actual_f/1e6} MHz")
                                await self.broadcast(json.dumps({"type": "status", "frequency": actual_f, "tuned": True}))
                            except:
                                await self.broadcast(json.dumps({"type": "error", "message": "Tuning Error"}))
                        else: self.freq = target_f

                    elif action == "setGain":
                        target_g = data.get("gain", self.gain)
                        if self.tb:
                            try:
                                self.tb.src.set_gain(target_g, 0)
                                actual_g = self.tb.src.get_gain(0)
                                self.gain = actual_g
                                print(f"[HARDWARE] Gain. Req: {target_g} | Got: {actual_g}")
                                await self.broadcast(json.dumps({"type": "status", "gain": actual_g}))
                            except: pass

                    elif action == "setRate":
                        new_rate = data.get("sampleRate", self.rate)
                        print(f"[SET] Sample Rate: {new_rate/1e3} kHz")
                        if self.tb:
                            self.rate = new_rate
                            self.start_hw() 
                    
                except Exception as e:
                    print(f"[WS] Interaction Error: {e}")
        finally:
            self.clients.remove(websocket)
            print(f"[WS] Client disconnected. Total: {len(self.clients)}")

async def main():
    bridge = USRPBridge()
    
    # Pre-flight check: Console feedback for hardware
    if USRP_AVAILABLE:
        try:
            found = uhd.find_devices()
            if found: print(f"[BOOT] USRP Detected: {found[0]}")
            else: print("[BOOT] WARNING: No hardware found.")
        except: pass

    # Run WS server and UDP listener concurrently
    server = websockets.serve(bridge.handle_ws, "localhost", WS_PORT)
    print(f"[MAIN] USRP Bridge v3.3 (Perfecto) on port {WS_PORT}")
    await asyncio.gather(server, bridge.udp_listener())

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
