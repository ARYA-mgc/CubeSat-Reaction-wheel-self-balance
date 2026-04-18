import os
import sys

# GNURadio Environment
gro_bin = r"C:\GNURadio-3.10\bin"
if os.path.exists(gro_bin):
    print(f"Adding {gro_bin} to DLL directory...")
    os.add_dll_directory(gro_bin)
    os.environ['PATH'] = gro_bin + ';' + os.environ['PATH']

print("Attempting to import gnuradio.uhd...")
try:
    from gnuradio import uhd
    print(f"SUCCESS: Found gnuradio.uhd at {uhd.__file__}")
    print(f"UHD Version: {uhd.uhd_sw_info()}")
except Exception as e:
    print(f"FAILED gnuradio.uhd: {e}")

print("\nAttempting to import uhd (standalone)...")
try:
    import uhd
    print(f"SUCCESS: Found uhd at {uhd.__file__}")
except Exception as e:
    print(f"FAILED standalone uhd: {e}")
