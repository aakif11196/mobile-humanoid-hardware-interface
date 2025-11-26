#!/usr/bin/env python3
"""
Simple UDP simulator to emulate NodeMCU wheel controllers.
Listens on a UDP port and replies to each ASCII float message with the same value
(or scaled by a multiplier). Useful for testing origin_hardware driver without NodeMCU.

Usage:
  python3 udp_sim.py [--port PORT] [--multiplier M]

Examples:
  python3 udp_sim.py --port 4210
  python3 udp_sim.py --port 4210 --multiplier 0.98
"""

import argparse
import socket
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--host', default='0.0.0.0', help='Interface to bind')
parser.add_argument('--port', type=int, default=80, help='UDP port to listen on')
parser.add_argument('--multiplier', type=float, default=1.0, help='Multiply incoming value before replying')
args = parser.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((args.host, args.port))
print(f"UDP simulator listening on {args.host}:{args.port} (multiplier={args.multiplier})")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        if not data:
            continue
        try:
            s = data.decode('ascii').strip()
        except Exception:
            s = ''
        print(f"recv from {addr}: '{s}'")
        try:
            v = float(s)
            out = f"{(v * args.multiplier):f}"
        except Exception:
            # not a float, echo raw
            out = s if s else "0.0"
        sock.sendto(out.encode('ascii'), addr)
        print(f"replied to {addr}: '{out}'")
except KeyboardInterrupt:
    print('shutting down')
    sock.close()
    sys.exit(0)
