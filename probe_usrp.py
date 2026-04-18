import sys
import os

# Inject paths
GR_ROOT = r"C:\GNURadio-3.10"
sys.path.insert(0, os.path.join(GR_ROOT, "lib", "site-packages"))
sys.path.insert(0, os.path.join(GR_ROOT, "python", "Lib"))
sys.path.insert(0, os.path.join(GR_ROOT, "bin"))

try:
    from gnuradio import uhd
    print(f"UHD Module: {uhd}")
    print(f"Attributes: {dir(uhd)}")
    
    # Check for submodules
    if hasattr(uhd, 'usrp'):
        print(f"uhd.usrp attributes: {dir(uhd.usrp)}")
        if hasattr(uhd.usrp, 'multi_usrp'):
            print("FOUND: uhd.usrp.multi_usrp")
    
    # Check for MultiUSRP-like classes
    for attr in dir(uhd):
        if 'multi' in attr.lower() or 'usrp' in attr.lower():
            obj = getattr(uhd, attr)
            print(f"Candidate '{attr}': {type(obj)}")
            
    # Check uhd_python specifically
    from gnuradio.uhd import uhd_python
    print(f"uhd_python Attributes: {dir(uhd_python)}")

except Exception as e:
    print(f"Error: {e}")
