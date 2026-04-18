import sys
import os

# Inject paths
GR_ROOT = r"C:\GNURadio-3.10"
sys.path.insert(0, os.path.join(GR_ROOT, "lib", "site-packages"))
sys.path.insert(0, os.path.join(GR_ROOT, "python", "Lib"))
sys.path.insert(0, os.path.join(GR_ROOT, "bin"))

def probe_obj(obj, name, depth=0, max_depth=2):
    if depth > max_depth: return
    try:
        attrs = dir(obj)
        for attr in attrs:
            if attr.startswith('_'): continue
            full_name = f"{name}.{attr}"
            child = getattr(obj, attr)
            
            # Check if this looks like a MultiUSRP
            if 'multi' in attr.lower() or 'usrp' in attr.lower():
                print(f"MATCH: {full_name} type: {type(child)}")
            
            # Check for methods
            if hasattr(child, 'get_rx_stream') or hasattr(child, 'recv'):
                print(f"FOUND STREAMER TARGET: {full_name} type: {type(child)}")
            
            if depth < max_depth and str(type(child)) == "<class 'module'>":
                probe_obj(child, full_name, depth + 1, max_depth)
    except: pass

try:
    from gnuradio import uhd
    print("--- Probing gnuradio.uhd ---")
    probe_obj(uhd, "uhd")
    
    from gnuradio.uhd import uhd_python
    print("\n--- Probing gnuradio.uhd.uhd_python ---")
    probe_obj(uhd_python, "uhd_python")

except Exception as e:
    print(f"Error: {e}")
