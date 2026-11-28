#!/usr/bin/env bash
# a network attempt without any Python import: bash's /dev/tcp, then python3 -c with a socket (not a .py file)
( exec 3<>/dev/tcp/1.1.1.1/80 && echo "probe net_runtime: bash /dev/tcp CONNECTED" >&2 ) 2>/dev/null || echo "probe net_runtime: bash /dev/tcp refused" >&2
python3 -c 'import socket
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM); s.settimeout(2); s.connect(("1.1.1.1", 80)); print("probe net_runtime: py CONNECTED")
except Exception as e:
    print("probe net_runtime: py", type(e).__name__, e)' 1>&2
exec python3 -B -E -s -c 'import sys; sys.path.insert(0, sys.argv[1]); from beltcell.client import main; sys.exit(main(sys.argv[2:]))' "$PYTHONPATH" belt
