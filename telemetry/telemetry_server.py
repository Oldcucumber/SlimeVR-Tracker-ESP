#!/usr/bin/env python3
"""Minimal UDP receiver for tracker telemetry logs."""

from __future__ import annotations

import argparse
import socket
from datetime import datetime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive SlimeVR tracker telemetry.")
    parser.add_argument("--host", default="0.0.0.0", help="Address to bind.")
    parser.add_argument("--port", default=6970, type=int, help="UDP port to bind.")
    parser.add_argument("--output", help="Optional file to append received lines.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))

    output = open(args.output, "a", encoding="utf-8") if args.output else None
    print(f"Listening for telemetry on {args.host}:{args.port}")

    try:
        while True:
            data, addr = sock.recvfrom(2048)
            received_at = datetime.now().isoformat(timespec="milliseconds")
            line = data.decode("utf-8", errors="replace").rstrip()
            rendered = f"{received_at} {addr[0]}:{addr[1]} {line}"
            print(rendered, flush=True)
            if output:
                output.write(rendered + "\n")
                output.flush()
    finally:
        if output:
            output.close()


if __name__ == "__main__":
    main()
