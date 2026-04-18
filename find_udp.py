import gnuradio
from gnuradio import gr
import sys

def find_in_mod(mod, mod_name, target='udp'):
    try:
        attrs = dir(mod)
        found = [x for x in attrs if target.lower() in x.lower()]
        if found:
            print(f"--- {mod_name} ---")
            for f in found:
                print(f"  {f}")
    except: pass

print(f"Python: {sys.version}")
print(f"GNURadio version: {gr.version()}")

# Check common modules
import gnuradio.gr as gr_mod
find_in_mod(gr_mod, "gnuradio.gr")

try:
    from gnuradio import blocks
    find_in_mod(blocks, "gnuradio.blocks")
except: print("gnuradio.blocks not found")

try:
    from gnuradio import network
    find_in_mod(network, "gnuradio.network")
except: print("gnuradio.network not found")

try:
    from gnuradio import zeromq
    find_in_mod(zeromq, "gnuradio.zeromq")
except: print("gnuradio.zeromq not found")
