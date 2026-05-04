"""Tests for app.protocol — frame encoding, decoding, builders."""

import pytest
from app.protocol import (
    SOF, Frame, PacketType, StatusCode,
    BadXorError, BadSofError, BadLengthError,
    decode_frame, read_exact_frame,
    build_read_response, build_write_response,
    parse_read_request, parse_write_request,
    le32_to_bytes, le32_from_bytes,
)


def make_iread_req(addr: int, tag: int, seq: int = 0x01) -> bytes:
    payload = le32_to_bytes(addr) + bytes([tag])
    f = Frame(packet_type=PacketType.IREAD_REQ, seq=seq, payload=payload)
    return f.encode()


def make_write_req(addr: int, wstrb: int, tag: int, data: int, seq: int = 0x02) -> bytes:
    payload = le32_to_bytes(addr) + bytes([wstrb, tag]) + le32_to_bytes(data)
    f = Frame(packet_type=PacketType.WRITE_REQ, seq=seq, payload=payload)
    return f.encode()


def test_encode_decode_iread():
    raw = make_iread_req(addr=0x00000104, tag=0x00, seq=0x07)
    f = decode_frame(raw)
    assert f.packet_type == PacketType.IREAD_REQ
    assert f.seq == 0x07
    assert f.length == 5
    addr, tag = parse_read_request(f.payload)
    assert addr == 0x00000104
    assert tag == 0x00


def test_encode_decode_write():
    raw = make_write_req(addr=0x00000104, wstrb=0xF, tag=0x01, data=0xDEADBEEF, seq=0x03)
    f = decode_frame(raw)
    assert f.packet_type == PacketType.WRITE_REQ
    addr, wstrb, tag, data = parse_write_request(f.payload)
    assert addr == 0x00000104
    assert wstrb == 0xF
    assert tag == 0x01
    assert data == 0xDEADBEEF


def test_bad_xor_raises():
    raw = bytearray(make_iread_req(0x100, 0x00))
    raw[-1] ^= 0xFF
    with pytest.raises(BadXorError):
        decode_frame(bytes(raw))


def test_bad_sof_raises():
    raw = bytearray(make_iread_req(0x100, 0x00))
    raw[0] = 0x00
    with pytest.raises(BadSofError):
        decode_frame(bytes(raw))


def test_too_short_raises():
    with pytest.raises(BadLengthError):
        decode_frame(b"\xA5\x01\x00")


def test_read_exact_frame_skips_garbage():
    good = make_iread_req(addr=0x0000_0000, tag=0x00, seq=0x05)
    junk = bytes([0x11, 0x22, 0x33])
    stream = iter(junk + good)
    f = read_exact_frame(lambda: next(stream))
    assert f.packet_type == PacketType.IREAD_REQ
    assert f.seq == 0x05


def test_build_read_response():
    resp = build_read_response(PacketType.IREAD_REQ, seq=0x07, status=StatusCode.OK,
                               tag=0x00, data=0xCAFEBABE)
    assert resp.packet_type == PacketType.IREAD_RESP
    assert resp.seq == 0x07
    assert resp.length == 6
    assert resp.payload[0] == StatusCode.OK
    assert resp.payload[1] == 0x00
    assert le32_from_bytes(resp.payload[2:]) == 0xCAFEBABE


def test_build_write_response():
    resp = build_write_response(seq=0x03, status=StatusCode.BAD_ADDR, tag=0x01)
    assert resp.packet_type == PacketType.WRITE_RESP
    assert resp.seq == 0x03
    assert resp.length == 2
    assert resp.payload[0] == StatusCode.BAD_ADDR
    assert resp.payload[1] == 0x01


def test_read_response_xor_is_valid():
    resp = build_read_response(PacketType.DREAD_REQ, seq=0xAB, status=0, tag=0, data=0x12345678)
    raw = resp.encode()
    decoded = decode_frame(raw)
    assert decoded.packet_type == PacketType.DREAD_RESP


def test_le32_roundtrip():
    for v in [0, 1, 0xDEADBEEF, 0xFFFFFFFF, 0x12345678]:
        assert le32_from_bytes(le32_to_bytes(v)) == v