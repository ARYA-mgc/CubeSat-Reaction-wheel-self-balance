from gnuradio import uhd
from gnuradio import gr
from gnuradio import blocks
import time
import numpy as np

class MiniBridge(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self, "Mini Bridge")
        
        # 1. Source
        st_args = uhd.stream_args("fc32", "sc16")
        self.src = uhd.usrp_source("type=b200", st_args)
        self.src.set_samp_rate(1e6)
        self.src.set_center_freq(2400e6, 0)
        self.src.set_gain(40, 0)
        
        # 2. Sink (Vector Sink)
        self.snk = blocks.vector_sink_c()
        
        # 3. Connect
        self.connect(self.src, self.snk)

try:
    print("[TEST] Creating Flowgraph...")
    tb = MiniBridge()
    tb.start()
    print("[TEST] Flowgraph Started. Waiting for data...")
    
    for _ in range(10):
        time.sleep(0.5)
        data = tb.snk.data()
        print(f"[TEST] Got {len(data)} samples")
        # Clear buffer if possible or just see it grows
        if len(data) > 0:
            break
            
    tb.stop()
    tb.wait()
    print("[TEST] Flowgraph Stopped.")

except Exception as e:
    print(f"Error: {e}")
