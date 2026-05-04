"""
UartMemoryAgent — services FPGA UART memory requests using MemoryModel.

Flow:
  1. FPGA sends IREAD_REQ / DREAD_REQ / WRITE_REQ frame
  2. Agent parses request, reads/writes MemoryModel
  3. Agent sends IREAD_RESP / DREAD_RESP / WRITE_RESP frame back

The ByteStream protocol requires only two methods:
  read_byte() -> int
  write_bytes(bytes) -> None
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol

from .memory_model import MemoryModel, MemoryRangeError
from .protocol import (
    PLEN_READ_REQ,
    PLEN_WRITE_REQ,
    Frame,
    PacketType,
    StatusCode,
    BadXorError,
    BadSofError,
    BadLengthError,
    build_read_response,
    build_write_response,
    parse_read_request,
    parse_write_request,
    read_exact_frame,
)


class ByteStream(Protocol):
    def read_byte(self) -> int: ...
    def write_bytes(self, data: bytes) -> None: ...


@dataclass
class AgentStats:
    frames_rx: int = 0
    frames_tx: int = 0
    iread_count: int = 0
    dread_count: int = 0
    write_count: int = 0
    bad_xor: int = 0
    bad_type: int = 0
    bad_len: int = 0
    bad_addr: int = 0
    started_at: float = field(default_factory=time.monotonic)

    def uptime(self) -> float:
        return time.monotonic() - self.started_at

    def __str__(self) -> str:
        return (
            f"uptime={self.uptime():.1f}s  "
            f"rx={self.frames_rx}  tx={self.frames_tx}  "
            f"iread={self.iread_count}  dread={self.dread_count}  "
            f"write={self.write_count}  "
            f"err(xor={self.bad_xor} type={self.bad_type} "
            f"len={self.bad_len} addr={self.bad_addr})"
        )


class UartMemoryAgent:
    def __init__(
        self,
        memory: MemoryModel,
        stream: ByteStream,
        verbose: bool = True,
    ) -> None:
        self.memory = memory
        self.stream = stream
        self.verbose = verbose
        self.stats = AgentStats()
        self._running = False

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[agent] {msg}")

    def _log_frame(self, direction: str, frame: Frame) -> None:
        if not self.verbose:
            return
        try:
            ptype_name = PacketType(frame.packet_type).name
        except ValueError:
            ptype_name = f"0x{frame.packet_type:02X}"
        self._log(
            f"{direction} {ptype_name} seq=0x{frame.seq:02X} "
            f"len={frame.length} payload=[{frame.payload.hex(' ')}] "
            f"xor=0x{frame.xor_checksum():02X}"
        )

    def _send(self, frame: Frame) -> None:
        self.stream.write_bytes(frame.encode())
        self.stats.frames_tx += 1
        self._log_frame("TX", frame)

    def _recv(self) -> Frame | None:
        try:
            frame = read_exact_frame(self.stream.read_byte)
            self.stats.frames_rx += 1
            self._log_frame("RX", frame)
            return frame
        except BadXorError as e:
            self.stats.bad_xor += 1
            self._log(f"ERROR {e}")
            return None
        except BadSofError as e:
            self._log(f"WARN  {e}")
            return None
        except BadLengthError as e:
            self.stats.bad_len += 1
            self._log(f"ERROR {e}")
            return None

    def handle_frame(self, frame: Frame) -> Frame:
        ptype = frame.packet_type

        if ptype in (PacketType.IREAD_REQ, PacketType.DREAD_REQ):
            if frame.length != PLEN_READ_REQ:
                self.stats.bad_len += 1
                self._log(
                    f"BAD LEN read req: got {frame.length}, expected {PLEN_READ_REQ}"
                )
                return build_read_response(ptype, frame.seq, StatusCode.BAD_TYPE, 0, 0)

            addr, tag = parse_read_request(frame.payload)

            try:
                data = self.memory.read_u32(addr)
                status = StatusCode.OK
            except MemoryRangeError:
                data = 0x0000_0000
                status = StatusCode.BAD_ADDR
                self.stats.bad_addr += 1

            if ptype == PacketType.IREAD_REQ:
                self.stats.iread_count += 1
                label = "IREAD"
            else:
                self.stats.dread_count += 1
                label = "DREAD"

            self._log(
                f"{label} addr=0x{addr:08X} tag=0x{tag:02X} "
                f"→ data=0x{data:08X} status={StatusCode(status).name}"
            )
            return build_read_response(ptype, frame.seq, int(status), tag, data)

        if ptype == PacketType.WRITE_REQ:
            if frame.length != PLEN_WRITE_REQ:
                self.stats.bad_len += 1
                self._log(
                    f"BAD LEN write req: got {frame.length}, expected {PLEN_WRITE_REQ}"
                )
                return build_write_response(frame.seq, StatusCode.BAD_TYPE, 0)

            addr, wstrb, tag, data = parse_write_request(frame.payload)

            try:
                self.memory.write_u32(addr, data, wstrb)
                status = StatusCode.OK
            except MemoryRangeError:
                status = StatusCode.BAD_ADDR
                self.stats.bad_addr += 1

            self.stats.write_count += 1
            self._log(
                f"WRITE addr=0x{addr:08X} wstrb=0b{wstrb:04b} "
                f"data=0x{data:08X} tag=0x{tag:02X} "
                f"→ status={StatusCode(status).name}"
            )
            return build_write_response(frame.seq, int(status), tag)

        self.stats.bad_type += 1
        self._log(f"UNKNOWN packet type=0x{ptype:02X}")
        return build_write_response(frame.seq, StatusCode.BAD_TYPE, 0)

    def serve_forever(self) -> None:
        self._running = True
        self._log(f"Started — memory size {len(self.memory)} bytes")
        try:
            while self._running:
                frame = self._recv()
                if frame is None:
                    continue
                response = self.handle_frame(frame)
                self._send(response)
        except KeyboardInterrupt:
            self._log("Stopped by user")
        finally:
            self._log(f"Stats: {self.stats}")

    def stop(self) -> None:
        self._running = False