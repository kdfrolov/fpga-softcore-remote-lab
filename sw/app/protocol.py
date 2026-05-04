"""
uart_agent_proto — Python-side implementation of the FPGA UART memory agent
protocol.

Frame layout (FPGA → PC, i.e. request from hardware agent):
  [SOF=0xA5][TYPE][SEQ][LEN][PAYLOAD x LEN][XOR]

  XOR = TYPE ^ SEQ ^ LEN ^ payload[0] ^ ... ^ payload[LEN-1]

Request types (FPGA → PC):
  0x01  IREAD_REQ   payload = ADDR[4LE] TAG[1]          len=5
  0x02  DREAD_REQ   payload = ADDR[4LE] TAG[1]          len=5
  0x03  WRITE_REQ   payload = ADDR[4LE] WSTRB[1] TAG[1] DATA[4LE]  len=10

Response types (PC → FPGA):
  0x81  IREAD_RESP  payload = STATUS[1] TAG[1] DATA[4LE]  len=6
  0x82  DREAD_RESP  payload = STATUS[1] TAG[1] DATA[4LE]  len=6
  0x83  WRITE_RESP  payload = STATUS[1] TAG[1]             len=2
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Callable


SOF: int = 0xA5


class PacketType(IntEnum):
    IREAD_REQ = 0x01
    DREAD_REQ = 0x02
    WRITE_REQ = 0x03
    IREAD_RESP = 0x81
    DREAD_RESP = 0x82
    WRITE_RESP = 0x83


class StatusCode(IntEnum):
    OK = 0x00
    BAD_XOR = 0x01
    BAD_TYPE = 0x02
    BAD_ADDR = 0x04
    BUSY = 0x06
    TIMEOUT = 0x07


PLEN_READ_REQ = 5
PLEN_WRITE_REQ = 10
PLEN_READ_RESP = 6
PLEN_WRITE_RESP = 2

_RESP_FOR_REQ: dict[int, int] = {
    PacketType.IREAD_REQ: PacketType.IREAD_RESP,
    PacketType.DREAD_REQ: PacketType.DREAD_RESP,
    PacketType.WRITE_REQ: PacketType.WRITE_RESP,
}


@dataclass(slots=True)
class Frame:
    packet_type: int
    seq: int
    payload: bytes

    @property
    def length(self) -> int:
        return len(self.payload)

    def xor_checksum(self) -> int:
        x = self.packet_type ^ self.seq ^ self.length
        for b in self.payload:
            x ^= b
        return x & 0xFF

    def encode(self) -> bytes:
        return bytes([
            SOF,
            self.packet_type & 0xFF,
            self.seq & 0xFF,
            self.length & 0xFF,
            *self.payload,
            self.xor_checksum(),
        ])

    def __repr__(self) -> str:
        try:
            ptype_name = PacketType(self.packet_type).name
        except ValueError:
            ptype_name = f"0x{self.packet_type:02X}"
        return (
            f"Frame(type={ptype_name}, seq=0x{self.seq:02X}, "
            f"len={self.length}, payload={self.payload.hex()}, "
            f"xor=0x{self.xor_checksum():02X})"
        )


class ProtocolError(Exception):
    pass


class BadSofError(ProtocolError):
    pass


class BadXorError(ProtocolError):
    pass


class BadLengthError(ProtocolError):
    pass


def _xor8(seq: int, ptype: int, length: int, payload: bytes) -> int:
    x = ptype ^ seq ^ length
    for b in payload:
        x ^= b
    return x & 0xFF


def decode_frame(raw: bytes) -> Frame:
    if len(raw) < 5:
        raise BadLengthError(f"Frame too short: {len(raw)} bytes")
    if raw[0] != SOF:
        raise BadSofError(f"Bad SOF: 0x{raw[0]:02X}, expected 0xA5")

    packet_type = raw[1]
    seq = raw[2]
    length = raw[3]
    expected_total = 5 + length

    if len(raw) != expected_total:
        raise BadLengthError(
            f"Frame size mismatch: got {len(raw)}, expected {expected_total}"
        )

    payload = bytes(raw[4:4 + length])
    got_xor = raw[-1]
    exp_xor = _xor8(seq, packet_type, length, payload)

    if got_xor != exp_xor:
        raise BadXorError(
            f"XOR mismatch: got 0x{got_xor:02X}, expected 0x{exp_xor:02X}"
        )

    return Frame(packet_type=packet_type, seq=seq, payload=payload)


def read_exact_frame(read_byte: Callable[[], int]) -> Frame:
    b = read_byte()
    while b != SOF:
        b = read_byte()

    packet_type = read_byte()
    seq = read_byte()
    length = read_byte()
    payload = bytes(read_byte() for _ in range(length))
    got_xor = read_byte()

    raw = bytes([SOF, packet_type, seq, length, *payload, got_xor])
    return decode_frame(raw)


def le32_to_bytes(value: int) -> bytes:
    return (value & 0xFFFF_FFFF).to_bytes(4, byteorder="little")


def le32_from_bytes(data: bytes | bytearray) -> int:
    return int.from_bytes(data[:4], byteorder="little")


def parse_read_request(payload: bytes) -> tuple[int, int]:
    if len(payload) != PLEN_READ_REQ:
        raise BadLengthError(
            f"Read request payload: expected {PLEN_READ_REQ} bytes, got {len(payload)}"
        )
    addr = le32_from_bytes(payload[0:4])
    tag = payload[4]
    return addr, tag


def parse_write_request(payload: bytes) -> tuple[int, int, int, int]:
    if len(payload) != PLEN_WRITE_REQ:
        raise BadLengthError(
            f"Write request payload: expected {PLEN_WRITE_REQ} bytes, got {len(payload)}"
        )
    addr = le32_from_bytes(payload[0:4])
    wstrb = payload[4]
    tag = payload[5]
    data = le32_from_bytes(payload[6:10])
    return addr, wstrb, tag, data


def build_read_response(req_type: int, seq: int, status: int, tag: int, data: int) -> Frame:
    resp_type = _RESP_FOR_REQ.get(req_type)
    if resp_type is None or req_type == PacketType.WRITE_REQ:
        raise ValueError(f"Not a read request type: 0x{req_type:02X}")
    payload = bytes([status & 0xFF, tag & 0xFF]) + le32_to_bytes(data)
    return Frame(packet_type=int(resp_type), seq=seq, payload=payload)


def build_write_response(seq: int, status: int, tag: int) -> Frame:
    payload = bytes([status & 0xFF, tag & 0xFF])
    return Frame(packet_type=int(PacketType.WRITE_RESP), seq=seq, payload=payload)