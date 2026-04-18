import sys
import os

# Setup DLL paths
uhd_bin = r"C:\Program Files\UHD\bin"
gro_bin = r"C:\GNURadio-3.10\bin"
if os.name == 'nt':
    if os.path.exists(uhd_bin):
        os.add_dll_directory(uhd_bin)
    if os.path.exists(gro_bin):
        os.add_dll_directory(gro_bin)

with open('uhd_probe.txt', 'w') as f:
    try:
        from gnuradio import uhd
        f.write(f"MODULE: {uhd}\n")
        f.write(f"ATTRIBUTES: {dir(uhd)}\n")
        f.write(f"FILE: {uhd.__file__}\n")
        if hasattr(uhd, 'usrp'):
            f.write(f"USRP_ATTRS: {dir(uhd.usrp)}\n")
    except Exception as e:
        f.write(f"ERROR: {e}\n")
