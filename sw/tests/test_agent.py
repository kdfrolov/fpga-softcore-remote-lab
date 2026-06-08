"""Integration tests: Agent + MemoryModel + MockByteStream"""

from app.agent import UartMemoryAgent
from app.memory_model import MemoryModel
from app.mock_stream import MockByteStream
from app.protocol import (
    Frame, PacketType, StatusCode,
    decode_frame, le32_to_bytes, le32_from_bytes,
)


def make_agent(mem_size: int = 4096 * 4) -> tuple[UartMemoryAgent, MemoryModel, MockByteStream]:
    memory = MemoryModel(size_bytes=mem_size)
    stream = MockByteStream()
    agent = UartMemoryAgent(memory=memory, stream=stream, verbose=False)
    return agent, memory, stream


def encode_iread_req(addr: int, tag: int, seq: int) -> bytes:
    payload = le32_to_bytes(addr) + bytes([tag])
    return Frame(PacketType.IREAD_REQ, seq=seq, payload=payload).encode()


def encode_dread_req(addr: int, tag: int, seq: int) -> bytes:
    payload = le32_to_bytes(addr) + bytes([tag])
    return Frame(PacketType.DREAD_REQ, seq=seq, payload=payload).encode()


def encode_write_req(addr: int, wstrb: int, tag: int, data: int, seq: int) -> bytes:
    payload = le32_to_bytes(addr) + bytes([wstrb, tag]) + le32_to_bytes(data)
    return Frame(PacketType.WRITE_REQ, seq=seq, payload=payload).encode()


def test_iread_ok():
    agent, memory, stream = make_agent()
    memory.write_u32(0x00, 0xDEADBEEF)

    req_frame = decode_frame(encode_iread_req(0x00, 0xAB, 0x01))
    resp = agent.handle_frame(req_frame)
    stream.write_bytes(resp.encode())

    tx = stream.take_tx()
    f = decode_frame(tx)
    assert f.packet_type == PacketType.IREAD_RESP
    assert f.seq == 0x01
    assert f.payload[0] == StatusCode.OK
    assert f.payload[1] == 0xAB
    assert le32_from_bytes(f.payload[2:]) == 0xDEADBEEF


def test_iread_bad_addr():
    agent, _, _ = make_agent(mem_size=64)
    req_frame = decode_frame(encode_iread_req(addr=0xFFFFFFFF, tag=0x00, seq=0x02))
    resp = agent.handle_frame(req_frame)
    assert resp.packet_type == PacketType.IREAD_RESP
    assert resp.payload[0] == StatusCode.BAD_ADDR


def test_dread_ok():
    agent, memory, _ = make_agent()
    memory.write_u32(0x104, 0x0000000A)
    req = decode_frame(encode_dread_req(addr=0x104, tag=0x01, seq=0x05))
    resp = agent.handle_frame(req)
    assert resp.packet_type == PacketType.DREAD_RESP
    assert resp.payload[0] == StatusCode.OK
    assert le32_from_bytes(resp.payload[2:]) == 0x0000000A


def test_write_ok_full_word():
    agent, memory, _ = make_agent()
    req = decode_frame(encode_write_req(addr=0x104, wstrb=0xF, tag=0x00, data=0x0000000A, seq=0x03))
    resp = agent.handle_frame(req)
    assert resp.packet_type == PacketType.WRITE_RESP
    assert resp.payload[0] == StatusCode.OK
    assert memory.read_u32(0x104) == 0x0000000A


def test_write_byte_wstrb1():
    agent, memory, _ = make_agent()
    memory.write_u32(0x100, 0x00000000)
    req = decode_frame(encode_write_req(addr=0x100, wstrb=0b0001, tag=0x00, data=0x000000FE, seq=0x04))
    agent.handle_frame(req)
    assert (memory.read_u32(0x100) & 0xFF) == 0xFE


def test_write_bad_addr():
    agent, _, _ = make_agent(mem_size=64)
    req = decode_frame(encode_write_req(addr=0xFFFF0000, wstrb=0xF, tag=0x00, data=0xFF, seq=0x05))
    resp = agent.handle_frame(req)
    assert resp.payload[0] == StatusCode.BAD_ADDR


def test_write_seq_echoed():
    agent, _, _ = make_agent()
    req = decode_frame(encode_write_req(addr=0x00, wstrb=0xF, tag=0x00, data=0x00, seq=0xEE))
    resp = agent.handle_frame(req)
    assert resp.seq == 0xEE


def test_unknown_type_returns_bad_type():
    agent, _, _ = make_agent()
    bad_frame = Frame(packet_type=0xFF, seq=0x01, payload=bytes([0x00]))
    resp = agent.handle_frame(bad_frame)
    assert resp.payload[0] == StatusCode.BAD_TYPE


def test_stats_increment():
    agent, memory, _ = make_agent()
    memory.write_u32(0x00, 0x1234)

    agent.handle_frame(decode_frame(encode_iread_req(0x00, 0, 1)))
    agent.handle_frame(decode_frame(encode_dread_req(0x00, 0, 2)))
    agent.handle_frame(decode_frame(encode_write_req(0x00, 0xF, 0, 0, 3)))

    assert agent.stats.iread_count == 1
    assert agent.stats.dread_count == 1
    assert agent.stats.write_count == 1