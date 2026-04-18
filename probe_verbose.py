
import os
import sys

# Add DLL paths
if os.name == 'nt':
    for p in [r"C:\Program Files\UHD\bin", r"C:\GNURadio-3.10\bin"]:
        if os.path.exists(p):
            os.add_dll_directory(p)

with open('uhd_probe_verbose.txt', 'w') as f:
    try:
        from gnuradio import uhd
        f.write(f"MODULE: {uhd}\n")
        if hasattr(uhd, 'uhd_python'):
            f.write(f"UHD_PYTHON ATTRS: {dir(uhd.uhd_python)}\n")
        else:
            f.write("uhd_python NOT FOUND in uhd\n")
            
        import gnuradio.uhd.uhd_python as up
        f.write(f"UP ATTRS: {dir(up)}\n")
        
    except Exception as e:
        f.write(f"ERROR: {e}\n")
