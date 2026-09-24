#!/usr/bin/env python3
"""Shared-session broker for the serial-over-TCP controller link.

The controller's bridge accepts ONE client at a time. This broker holds that
single connection and lets several local clients share it, so a human and
Claude can work in the same session and see each other's commands.

    ssh -N -o ServerAliveInterval=30 -L 5000:<controller-ip>:5000 <jump-host>   # tunnel
    python3 python/net_broker.py            # this file, leave running
    python3 python/net_client.py            # interactive terminal (you)
    python3 python/net_client.py --name claude --send "*IDN?"   # one-shot (Claude)

Everything the controller sends goes to every client. Each line a client sends
goes to the controller and is shown to the OTHER clients as "[name] > line".
The whole dialog is also appended to logs/net_session.log.
"""
import argparse
import asyncio
import os
import time

ap = argparse.ArgumentParser()
ap.add_argument("--upstream", default="localhost:5000", help="tunnel host:port")
ap.add_argument("--listen-port", type=int, default=5001)
ap.add_argument("--log", default="logs/net_session.log")
args = ap.parse_args()

up_host, up_port = args.upstream.rsplit(":", 1)
clients = {}          # writer -> name
up_writer = None
os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
logf = open(args.log, "a", buffering=1)


def log(text):
    logf.write(f"{time.strftime('%H:%M:%S')} {text}\n")


def broadcast(data, exclude=None):
    for w in list(clients):
        if w is not exclude:
            try:
                w.write(data)
            except Exception:
                clients.pop(w, None)


async def handle_client(reader, writer):
    name = "client"
    try:
        first = await reader.readline()
        if first.startswith(b"@name "):
            name = first[6:].decode(errors="replace").strip() or name
            first = b""
        clients[writer] = name
        log(f"--- {name} joined ---")
        broadcast(f"--- {name} joined ---\r\n".encode(), exclude=writer)
        writer.write(f"--- connected as {name}; {len(clients)} client(s) ---\r\n".encode())
        pending = first
        while True:
            line = pending or await reader.readline()
            pending = b""
            if not line:
                break
            text = line.decode(errors="replace").rstrip("\r\n")
            log(f"[{name}] > {text}")
            broadcast(f"[{name}] > {text}\r\n".encode(), exclude=writer)
            up_writer.write(text.encode() + b"\r\n")
            await up_writer.drain()
    finally:
        clients.pop(writer, None)
        log(f"--- {name} left ---")
        broadcast(f"--- {name} left ---\r\n".encode())
        writer.close()


async def pump_upstream(reader):
    buf = b""
    while True:
        data = await reader.read(4096)
        if not data:
            log("--- upstream closed ---")
            broadcast(b"--- controller connection closed ---\r\n")
            os._exit(1)
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            log(line.decode(errors="replace").rstrip("\r"))
        broadcast(data)


async def main():
    global up_writer
    ur, up_writer = await asyncio.open_connection(up_host, int(up_port))
    print(f"upstream {args.upstream} connected; clients on 127.0.0.1:{args.listen_port}; log {args.log}")
    server = await asyncio.start_server(handle_client, "127.0.0.1", args.listen_port)
    await asyncio.gather(pump_upstream(ur), server.serve_forever())


try:
    asyncio.run(main())
except KeyboardInterrupt:
    pass
