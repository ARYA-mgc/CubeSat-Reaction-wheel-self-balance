import sys
import os

# Setup DLL paths first
uhd_bin = r"C:\Program Files\UHD\bin"
gro_bin = r"C:\GNURadio-3.10\bin"
if os.path.exists(uhd_bin):
    os.add_dll_directory(uhd_bin)
if os.path.exists(gro_bin):
    os.add_dll_directory(gro_bin)

try:
    import uhd
    print(f"UHD Module: {uhd}")
    print(f"Attributes: {dir(uhd)}")
    if hasattr(uhd, 'usrp'):
        print(f"UHD.USRP Attributes: {dir(uhd.usrp)}")
except Exception as e:
    print(f"Direct import failed: {e}")

try:
    from gnuradio import uhd as gro_uhd
    print(f"GNURadio UHD Module: {gro_uhd}")
    print(f"Attributes: {dir(gro_uhd)}")
except Exception as e:
    print(f"GNURadio import failed: {e}")
