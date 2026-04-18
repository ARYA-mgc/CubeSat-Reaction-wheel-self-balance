import os
import sys

# List of potential DLL directories
dll_dirs = [
    r"C:\GNURadio-3.10\bin",
    r"C:\Program Files\UHD\bin",
    r"C:\Program Files\NI\NI-USRP", # Sometimes NI installs here
    r"C:\Windows\System32"
]

if os.name == 'nt':
    print("--- Setting up DLL Directories ---")
    for d in dll_dirs:
        if os.path.exists(d):
            print(f"Adding: {d}")
            try:
                os.add_dll_directory(d)
                os.environ['PATH'] = d + ';' + os.environ['PATH']
            except Exception as e:
                print(f"Failed to add {d}: {e}")

print("\n--- Testing Imports ---")
try:
    print("Importing gnuradio.uhd...")
    from gnuradio import uhd
    print("SUCCESS: gnuradio.uhd imported.")
    print(f"UHD Version: {uhd.uhd_sw_info()}")
    sys.exit(0)
except Exception as e:
    print(f"FAILED gnuradio.uhd: {e}")
    import traceback
    traceback.print_exc()

try:
    print("\nImporting standalone uhd...")
    import uhd
    print("SUCCESS: standalone uhd imported.")
    sys.exit(0)
except Exception as e:
    print(f"FAILED standalone uhd: {e}")

sys.exit(1)
