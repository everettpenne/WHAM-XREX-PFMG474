#!/usr/bin/env python3
"""Minimal always-listening terminal for a serial-over-TCP controller link.

Start the tunnel first (separate terminal):
    ssh -N -o ServerAliveInterval=30 -L 5000:<controller-ip>:5000 <jump-host>

Then:
    python3 python/net_terminal.py [host] [port]      (default localhost 5000)

Everything the controller sends is printed as it arrives; each line you
type is sent with CRLF. Ctrl-D or Ctrl-C to quit.
"""
import socket
import sys
import threading

host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 5000

sock = socket.create_connection((host, port), timeout=5)
sock.settimeout(None)
print(f"--- connected to {host}:{port} (Ctrl-D to quit) ---")


def reader():
    while True:
        data = sock.recv(4096)
        if not data:
            print("\n--- connection closed by remote ---")
            break
        sys.stdout.write(data.decode(errors="replace"))
        sys.stdout.flush()


threading.Thread(target=reader, daemon=True).start()

try:
    for line in sys.stdin:
        sock.sendall(line.rstrip("\r\n").encode() + b"\r\n")
except (KeyboardInterrupt, BrokenPipeError):
    pass
finally:
    sock.close()
