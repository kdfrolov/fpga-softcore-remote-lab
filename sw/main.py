#!/usr/bin/env python3
"""
Entry point for the desktop FPGA PC-side tools.

Modes:
- GUI (default): python main.py
- Headless memory-agent: python main.py serve --port COM3 --baud 115200 --hex program.hex
"""

from __future__ import annotations

import argparse
import sys

from app.agent import UartMemoryAgent
from app.memory_model import MemoryModel
from app.serial_port import SerialByteStream
from app.ui import launch_ui


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FPGA PC-side tools")
    sub = parser.add_subparsers(dest="mode")

    gui = sub.add_parser("gui", help="Launch desktop UI")
    gui.add_argument("--baud", type=int, default=115200, help="Default UART baud rate")
    gui.add_argument("--mem", type=int, default=4096 * 4, help="Default memory size in bytes")
    gui.add_argument("--hex", default=None, help="Optional program.hex to preload in GUI")

    serve = sub.add_parser("serve", help="Run UART memory agent without GUI")
    serve.add_argument("--port", required=True, help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    serve.add_argument("--baud", type=int, default=115200, help="Baud rate")
    serve.add_argument("--mem", type=int, default=4096 * 4, help="Memory size in bytes")
    serve.add_argument("--hex", default=None, help="Plain hex word file to preload")
    serve.add_argument("--quiet", action="store_true", help="Suppress per-frame log output")

    return parser


def run_headless(args: argparse.Namespace) -> int:
    memory = MemoryModel(size_bytes=args.mem)

    if args.hex:
        words = memory.load_hex_words(args.hex)
        print(f"[main] Loaded {words} words from {args.hex!r}")

    print(f"[main] Opening {args.port} @ {args.baud} baud ...")
    try:
        stream = SerialByteStream(port=args.port, baud=args.baud, timeout=0.2)
    except Exception as exc:
        print(f"[main] Failed to open port: {exc}", file=sys.stderr)
        return 1

    agent = UartMemoryAgent(memory=memory, stream=stream, verbose=not args.quiet)

    try:
        with stream:
            agent.serve_forever()
    except KeyboardInterrupt:
        pass

    return 0


def main() -> None:
    parser = build_parser()

    argv = sys.argv[1:]
    if not argv:
        argv = ["gui"]
    elif argv[0] not in {"gui", "serve"}:
        argv = ["gui", *argv]

    args = parser.parse_args(argv)

    if args.mode == "serve":
        raise SystemExit(run_headless(args))

    launch_ui(
        default_mem_size=getattr(args, "mem", 4096 * 4),
        default_baud=getattr(args, "baud", 115200),
        default_hex=getattr(args, "hex", None),
    )


if __name__ == "__main__":
    main()