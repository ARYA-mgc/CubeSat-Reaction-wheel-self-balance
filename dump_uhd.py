
import os
import sys

# Add DLL paths
if os.name == 'nt':
    for p in [r"C:\Program Files\UHD\bin", r"C:\GNURadio-3.10\bin"]:
        if os.path.exists(p):
            os.add_dll_directory(p)

with open('uhd_attrs.txt', 'w') as f:
    try:
        from gnuradio import uhd
        f.write(f"MODULE: {uhd}\n")
        f.write(f"FILE: {uhd.__file__}\n")
        f.write("ATTRIBUTES:\n")
        for attr in sorted(dir(uhd)):
            f.write(f"  {attr}\n")
            
        if hasattr(uhd, 'usrp'):
            f.write("\nUSRP ATTRIBUTES:\n")
            for attr in sorted(dir(uhd.usrp)):
                f.write(f"  {attr}\n")
    except Exception as e:
        f.write(f"ERROR: {e}\n")
