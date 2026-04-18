import sys
import os

print(f"Python: {sys.version}")
print(f"Path: {sys.path}")

try:
    import gnuradio
    print(f"GNURadio version: {gnuradio.__version__ if hasattr(gnuradio, '__version__') else 'Found'}")
except ImportError:
    print("gnuradio not found")

try:
    from gnuradio import uhd
    print(f"gnuradio.uhd found: {uhd}")
    print(f"Methods: {dir(uhd)}")
except ImportError as e:
    print(f"gnuradio.uhd not found: {e}")

try:
    import uhd
    print(f"standalone uhd found: {uhd}")
except ImportError as e:
    print(f"standalone uhd not found: {e}")
