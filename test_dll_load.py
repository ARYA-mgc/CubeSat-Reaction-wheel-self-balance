import os
import sys
import ctypes

def test_dll(path, name):
    full_path = os.path.join(path, name)
    if not os.path.exists(full_path):
        print(f"NOT FOUND: {full_path}")
        return False
    try:
        # Load with dependencies
        print(f"Attempting to load {full_path}...")
        lib = ctypes.CDLL(full_path)
        print(f"SUCCESSfully loaded {name} from {path}")
        return True
    except Exception as e:
        print(f"FAILED to load {name} from {path}: {e}")
        return False

uhd_bin = r"C:\Program Files\UHD\bin"
gro_bin = r"C:\GNURadio-3.10\bin"

# Add to DLL search path for Python 3.8+
if os.name == 'nt':
    for p in [uhd_bin, gro_bin]:
        if os.path.exists(p):
            print(f"Adding {p} to DLL directory...")
            os.add_dll_directory(p)
            os.environ['PATH'] = p + ';' + os.environ['PATH']

print("\n--- DLL Test ---")
test_dll(uhd_bin, "libusb-1.0.dll")
test_dll(uhd_bin, "uhd.dll")
test_dll(gro_bin, "uhd.dll")
test_dll(gro_bin, "gnuradio-uhd.dll")

print("\n--- Python Import Test ---")
try:
    from gnuradio import uhd
    print("SUCCESS: from gnuradio import uhd")
except Exception as e:
    print(f"FAILED: from gnuradio import uhd -> {e}")
