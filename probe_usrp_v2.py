import sys
import os

# Inject paths
GR_ROOT = r"C:\GNURadio-3.10"
sys.path.insert(0, os.path.join(GR_ROOT, "lib", "site-packages"))
sys.path.insert(0, os.path.join(GR_ROOT, "python", "Lib"))
sys.path.insert(0, os.path.join(GR_ROOT, "bin"))

try:
    print("--- Searching in gnuradio.uhd ---")
    from gnuradio import uhd
    for attr in dir(uhd):
        if 'multi' in attr.lower() or 'usrp' in attr.lower():
            print(f"uhd.{attr} type: {type(getattr(uhd, attr))}")

    print("\n--- Searching in gnuradio.uhd.uhd_python ---")
    from gnuradio.uhd import uhd_python
    for attr in dir(uhd_python):
        if 'multi' in attr.lower() or 'usrp' in attr.lower():
            print(f"uhd_python.{attr} type: {type(getattr(uhd_python, attr))}")
            if attr == 'multi_usrp':
                print(f"multi_usrp Methods: {dir(getattr(uhd_python, attr))}")

except Exception as e:
    print(f"Error: {e}")
