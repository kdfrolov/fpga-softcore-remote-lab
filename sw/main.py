#!/usr/bin/env python3
"""
Entry point: UART memory agent server.

Usage:
    python main.py --port COM3 --baud 115200 --hex data/program.hex
    python main.py --port /dev/ttyUSB0 --baud 115200 --hex data/program.hex
"""

import argparse
import sys

from app.agent import UartMemoryAgent
from app.memory_model import MemoryModel
from app.serial_port import SerialByteStream


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="FPGA UART memory agent (PC side)")
    p.add_argument("--port", required=True, help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--mem", type=int, default=4096 * 4, help="Memory size in bytes (default: 16384)")
    p.add_argument("--hex", default=None, help="Plain hex word file to preload")
    p.add_argument("--quiet", action="store_true", help="Suppress per-frame log output")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    memory = MemoryModel(size_bytes=args.mem)

    if args.hex:
        n = memory.load_hex_words(args.hex)
        print(f"[main] Loaded {n} words from {args.hex!r}")

    print(f"[main] Opening {args.port} @ {args.baud} baud ...")
    try:
        stream = SerialByteStream(port=args.port, baud=args.baud)
    except Exception as e:
        print(f"[main] Failed to open port: {e}", file=sys.stderr)
        sys.exit(1)

    agent = UartMemoryAgent(memory=memory, stream=stream, verbose=not args.quiet)

    with stream:
        agent.serve_forever()


if __name__ == "__main__":
    main()