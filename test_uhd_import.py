import sys
import os

# Try to find a standalone 'uhd' module
GR_ROOT = r"C:\GNURadio-3.10"
search_paths = [
    os.path.join(GR_ROOT, "lib", "site-packages"),
    os.path.join(GR_ROOT, "lib", "site-packages", "gnuradio"),
]

for p in search_paths:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    import uhd
    print(f"SUCCESS: Imported 'uhd' from {uhd.__file__}")
    print(f"uhd dir: {dir(uhd)}")
except ImportError as e:
    print(f"ImportError: {e}")
except Exception as e:
    print(f"Error: {e}")

try:
    from gnuradio import uhd as gr_uhd
    print(f"SUCCESS: Imported 'gnuradio.uhd' from {gr_uhd.__file__}")
    print(f"gr_uhd dir: {dir(gr_uhd)}")
    
    # Try to see if it has 'usrp'
    if hasattr(gr_uhd, 'usrp'):
        print(f"gr_uhd.usrp: {gr_uhd.usrp}")
except ImportError as e:
    print(f"ImportError (GR): {e}")
