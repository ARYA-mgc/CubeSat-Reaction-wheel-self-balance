
import os
import sys

# DLL setup
for p in [r"C:\Program Files\UHD\bin", r"C:\GNURadio-3.10\bin"]:
    if os.path.exists(p):
        os.add_dll_directory(p)

try:
    from gnuradio import uhd
    print(f"MODULE: {uhd}")
    print("TOP LEVEL ATTRS:", [a for a in dir(uhd) if not a.startswith('__')])
    
    # Check for uhd_python submodule
    import gnuradio.uhd.uhd_python as up
    print("uhd_python ATTRS:", [a for a in dir(up) if not a.startswith('__')])
    
except Exception as e:
    print(f"PROBE FAILED: {e}")
