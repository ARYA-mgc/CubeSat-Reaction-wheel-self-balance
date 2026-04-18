from gnuradio import uhd
import sys

try:
    # 1. Create source
    st_args = uhd.stream_args("fc32", "sc16")
    src = uhd.usrp_source("type=b200", st_args)
    print(f"Source: {src}")
    
    # 2. Get device
    if hasattr(src, 'get_device'):
        dev = src.get_device()
        print(f"Device: {dev}")
        print(f"Device Methods: {[x for x in dir(dev) if 'stream' in x or 'recv' in x]}")
        
        # 3. Test streamer
        if hasattr(dev, 'get_rx_stream'):
            streamer = dev.get_rx_stream(st_args)
            print(f"Streamer: {streamer}")
    else:
        print("NO GET_DEVICE")

except Exception as e:
    print(f"Error: {e}")
