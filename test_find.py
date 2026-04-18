
import os
import sys

# DLL setup
for p in [r"C:\Program Files\UHD\bin", r"C:\GNURadio-3.10\bin"]:
    if os.path.exists(p):
        os.add_dll_directory(p)

try:
    from gnuradio import uhd
    print("SEARCHING FOR DEVICES...")
    devs = uhd.find_devices()
    print(f"DEVICES FOUND: {list(devs)}")
    
    # Try creating a usrp_block
    try:
        print("TRYING usrp_block.make...")
        # addr = uhd.device_addr("type=b200")
        # usrp = uhd.usrp_block.make(addr)
        # print(f"USRP BLOCK: {usrp}")
    except Exception as e:
        print(f"usrp_block failed: {e}")

except Exception as e:
    print(f"TEST FAILED: {e}")
