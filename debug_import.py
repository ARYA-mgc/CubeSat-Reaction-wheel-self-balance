import sys
import os

print("Python Executable:", sys.executable)
print("System Path:")
for p in sys.path:
    print("  ", p)

print("\nAttempting to import gnuradio...")
try:
    import gnuradio
    print("SUCCESS: imported gnuradio from", gnuradio.__file__)
except ImportError as e:
    print("FAILED: import gnuradio:", e)

print("\nAttempting to import gnuradio.uhd...")
try:
    from gnuradio import uhd
    print("SUCCESS: imported gnuradio.uhd")
except ImportError as e:
    print("FAILED: import gnuradio.uhd:", e)

print("\nAttempting to import uhd (direct)...")
try:
    import uhd
    print("SUCCESS: imported uhd (direct)")
except ImportError as e:
    print("FAILED: import uhd (direct):", e)
