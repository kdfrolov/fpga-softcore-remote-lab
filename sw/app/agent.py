"""UART memory agent with bidirectional control-command support"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol

from .memory_model import MemoryModel, MemoryRangeError
from .protocol import (
    PLEN_READ_REQ,
    PLEN_WRITE_REQ,
    BadLengthError,
    BadSofError,
    BadXorError,
    Frame,
    PacketType,
    StatusCode,
    build_core_reset_request,
    build_read_response,
    build_set_clkdiv_request,
    build_set_hold_request,
    build_set_regsel_request,
    build_write_response,
    expected_response_type,
    parse_core_reset_response,
    parse_read_request,
    parse_set_clkdiv_response,
    parse_set_hold_response,
    parse_set_regsel_response,
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
    control_tx_count: int = 0
    control_resp_count: int = 0
    bad_xor: int = 0
    bad_type: int = 0
    bad_len: int = 0
    bad_addr: int = 0
    started_at: float = field(default_factory=time.monotonic)

    def uptime(self) -> float:
        return time.monotonic() - self.started_at

    def __str__(self) -> str:
        return (
            f"uptime={self.uptime():.1f}s rx={self.frames_rx} tx={self.frames_tx} "
            f"iread={self.iread_count} dread={self.dread_count} write={self.write_count} "
            f"ctrl_tx={self.control_tx_count} ctrl_resp={self.control_resp_count} "
            f"err(xor={self.bad_xor} type={self.bad_type} len={self.bad_len} addr={self.bad_addr})"
        )


@dataclass
class ControlCommand:
    name: str
    frame: Frame
    expected_resp_type: int
    timeout_s: float = 1.0


@dataclass
class ControlResult:
    ok: bool
    status: int
    message: str
    response_value: int | None = None
    response_frame: Frame | None = None


@dataclass
class _PendingControl:
    command: ControlCommand
    event: threading.Event = field(default_factory=threading.Event)
    result: ControlResult | None = None


class UartMemoryAgent:
    def __init__(
        self,
        memory: MemoryModel,
        stream: ByteStream,
        verbose: bool = True,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.memory = memory
        self.stream = stream
        self.verbose = verbose
        self.logger = logger
        self.stats = AgentStats()
        self._running = False
        self._cmd_seq = 0
        self._command_queue: queue.Queue[_PendingControl] = queue.Queue()
        self._active_command: _PendingControl | None = None
        self._active_deadline: float = 0.0
        self._lock = threading.Lock()

    def _log(self, msg: str) -> None:
        if self.logger is not None:
            self.logger(msg)
        elif self.verbose:
            print(f"[agent] {msg}")

    def _log_frame(self, direction: str, frame: Frame) -> None:
        if not self.verbose and self.logger is None:
            return
        try:
            ptype_name = PacketType(frame.packet_type).name
        except ValueError:
            ptype_name = f"0x{frame.packet_type:02X}"
        self._log(
            f"{direction} {ptype_name} seq=0x{frame.seq:02X} len={frame.length} "
            f"payload=[{frame.payload.hex(' ')}] xor=0x{frame.xor_checksum():02X}"
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
        except TimeoutError:
            return None
        except BadXorError as exc:
            self.stats.bad_xor += 1
            self._log(f"ERROR {exc}")
            return None
        except BadSofError as exc:
            self._log(f"WARN {exc}")
            return None
        except BadLengthError as exc:
            self.stats.bad_len += 1
            self._log(f"ERROR {exc}")
            return None

    def _status_name(self, status: int) -> str:
        try:
            return StatusCode(status).name
        except ValueError:
            return f"0x{status:02X}"

    def handle_memory_frame(self, frame: Frame) -> Frame:
        ptype = frame.packet_type

        if ptype in (PacketType.IREAD_REQ, PacketType.DREAD_REQ):
            if frame.length != PLEN_READ_REQ:
                self.stats.bad_len += 1
                self._log(f"BAD LEN read req: got {frame.length}, expected {PLEN_READ_REQ}")
                return build_read_response(ptype, frame.seq, StatusCode.BAD_TYPE, 0, 0)

            addr, tag = parse_read_request(frame.payload)
            try:
                data = self.memory.read_u32(addr)
                status = StatusCode.OK
            except MemoryRangeError:
                data = 0
                status = StatusCode.BAD_ADDR
                self.stats.bad_addr += 1

            if ptype == PacketType.IREAD_REQ:
                self.stats.iread_count += 1
                label = "IREAD"
            else:
                self.stats.dread_count += 1
                label = "DREAD"

            self._log(
                f"{label} addr=0x{addr:08X} tag=0x{tag:02X} -> data=0x{data:08X} "
                f"status={self._status_name(int(status))}"
            )
            return build_read_response(ptype, frame.seq, int(status), tag, data)

        if ptype == PacketType.WRITE_REQ:
            if frame.length != PLEN_WRITE_REQ:
                self.stats.bad_len += 1
                self._log(f"BAD LEN write req: got {frame.length}, expected {PLEN_WRITE_REQ}")
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
                f"WRITE addr=0x{addr:08X} wstrb=0b{wstrb:04b} data=0x{data:08X} "
                f"tag=0x{tag:02X} -> status={self._status_name(int(status))}"
            )
            return build_write_response(frame.seq, int(status), tag)

        self.stats.bad_type += 1
        self._log(f"UNKNOWN packet type=0x{ptype:02X}")
        return build_write_response(frame.seq, StatusCode.BAD_TYPE, 0)

    def _parse_control_response(self, pending: _PendingControl, frame: Frame) -> ControlResult:
        ptype = frame.packet_type
        if ptype == PacketType.SET_CLKDIV_RESP:
            status, div = parse_set_clkdiv_response(frame.payload)
            return ControlResult(
                ok=status == StatusCode.OK,
                status=status,
                message=f"SET_CLKDIV -> status={self._status_name(status)}, div={div}",
                response_value=div,
                response_frame=frame,
            )
        if ptype == PacketType.SET_HOLD_RESP:
            status, hold = parse_set_hold_response(frame.payload)
            return ControlResult(
                ok=status == StatusCode.OK,
                status=status,
                message=f"SET_HOLD -> status={self._status_name(status)}, hold={hold}",
                response_value=hold,
                response_frame=frame,
            )
        if ptype == PacketType.CORE_RESET_RESP:
            status = parse_core_reset_response(frame.payload)
            return ControlResult(
                ok=status == StatusCode.OK,
                status=status,
                message=f"CORE_RESET -> status={self._status_name(status)}",
                response_value=None,
                response_frame=frame,
            )
        if ptype == PacketType.SET_REGSEL_RESP:
            status, regsel = parse_set_regsel_response(frame.payload)
            return ControlResult(
                ok=status == StatusCode.OK,
                status=status,
                message=f"SET_REGSEL -> status={self._status_name(status)}, regsel={regsel}",
                response_value=regsel,
                response_frame=frame,
            )
        return ControlResult(False, int(StatusCode.BAD_TYPE), f"Unexpected response type 0x{ptype:02X}", None, frame)

    def _dispatch_control(self) -> None:
        if self._active_command is not None:
            return
        try:
            pending = self._command_queue.get_nowait()
        except queue.Empty:
            return
        self._active_command = pending
        self._active_deadline = time.monotonic() + pending.command.timeout_s
        self.stats.control_tx_count += 1
        self._send(pending.command.frame)
        self._log(f"CONTROL {pending.command.name} sent")

    def _finish_active_command(self, result: ControlResult) -> None:
        if self._active_command is None:
            return
        self._active_command.result = result
        self._active_command.event.set()
        self._log(result.message)
        self._active_command = None
        self._active_deadline = 0.0

    def _check_active_timeout(self) -> None:
        if self._active_command is None:
            return
        if time.monotonic() < self._active_deadline:
            return
        self._finish_active_command(
            ControlResult(
                ok=False,
                status=int(StatusCode.TIMEOUT),
                message=f"{self._active_command.command.name} -> status=TIMEOUT",
                response_value=None,
                response_frame=None,
            )
        )

    def submit_control(self, command: ControlCommand) -> ControlResult:
        if not self._running:
            raise RuntimeError("Agent is not running")
        pending = _PendingControl(command=command)
        self._command_queue.put(pending)
        completed = pending.event.wait(timeout=command.timeout_s + 0.5)
        if not completed or pending.result is None:
            return ControlResult(False, int(StatusCode.TIMEOUT), f"{command.name} -> status=TIMEOUT")
        return pending.result

    def _next_seq(self) -> int:
        with self._lock:
            seq = self._cmd_seq & 0xFF
            self._cmd_seq = (self._cmd_seq + 1) & 0xFF
            return seq

    def set_clk_div(self, div: int, timeout_s: float = 1.0) -> ControlResult:
        if not 0 <= div <= 15:
            raise ValueError("Clock divider must be in range 0..15")
        seq = self._next_seq()
        return self.submit_control(
            ControlCommand(
                name=f"SET_CLKDIV({div})",
                frame=build_set_clkdiv_request(seq, div),
                expected_resp_type=expected_response_type(PacketType.SET_CLKDIV_REQ),
                timeout_s=timeout_s,
            )
        )

    def set_hold(self, hold: bool, timeout_s: float = 1.0) -> ControlResult:
        seq = self._next_seq()
        return self.submit_control(
            ControlCommand(
                name=f"SET_HOLD({int(bool(hold))})",
                frame=build_set_hold_request(seq, hold),
                expected_resp_type=expected_response_type(PacketType.SET_HOLD_REQ),
                timeout_s=timeout_s,
            )
        )

    def core_reset(self, timeout_s: float = 1.0) -> ControlResult:
        seq = self._next_seq()
        return self.submit_control(
            ControlCommand(
                name="CORE_RESET",
                frame=build_core_reset_request(seq),
                expected_resp_type=expected_response_type(PacketType.CORE_RESET_REQ),
                timeout_s=timeout_s,
            )
        )

    def set_regsel(self, regsel: int, timeout_s: float = 1.0) -> ControlResult:
        if not 0 <= regsel <= 31:
            raise ValueError("Register select must be in range 0..31")
        seq = self._next_seq()
        return self.submit_control(
            ControlCommand(
                name=f"SET_REGSEL({regsel})",
                frame=build_set_regsel_request(seq, regsel),
                expected_resp_type=expected_response_type(PacketType.SET_REGSEL_REQ),
                timeout_s=timeout_s,
            )
        )

    def serve_forever(self) -> None:
        self._running = True
        self._log(f"Started - memory size {len(self.memory)} bytes")
        try:
            while self._running:
                self._dispatch_control()
                frame = self._recv()
                if frame is not None:
                    if frame.packet_type in (PacketType.IREAD_REQ, PacketType.DREAD_REQ, PacketType.WRITE_REQ):
                        response = self.handle_memory_frame(frame)
                        self._send(response)
                    elif (
                        self._active_command is not None
                        and frame.seq == self._active_command.command.frame.seq
                        and frame.packet_type == self._active_command.command.expected_resp_type
                    ):
                        self.stats.control_resp_count += 1
                        result = self._parse_control_response(self._active_command, frame)
                        self._finish_active_command(result)
                    else:
                        self._log(f"IGNORED frame type=0x{frame.packet_type:02X} seq=0x{frame.seq:02X}")
                self._check_active_timeout()
        except KeyboardInterrupt:
            self._log("Stopped by user")
        finally:
            self._running = False
            if self._active_command is not None:
                self._finish_active_command(ControlResult(False, int(StatusCode.TIMEOUT), "Agent stopped"))
            while True:
                try:
                    pending = self._command_queue.get_nowait()
                except queue.Empty:
                    break
                pending.result = ControlResult(False, int(StatusCode.TIMEOUT), "Agent stopped")
                pending.event.set()
            self._log(f"Stats: {self.stats}")

    def stop(self) -> None:
        self._running = False