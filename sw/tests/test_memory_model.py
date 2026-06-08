"""Tests for memory_model"""

import pytest
from app.memory_model import MemoryModel, MemoryRangeError


def make_mem(size: int = 64) -> MemoryModel:
    return MemoryModel(size_bytes=size)


def test_read_write_u32():
    m = make_mem()
    m.write_u32(0, 0xDEADBEEF)
    assert m.read_u32(0) == 0xDEADBEEF


def test_multiple_words():
    m = make_mem(size=64)
    for i in range(16):
        m.write_u32(i * 4, i)
    for i in range(16):
        assert m.read_u32(i * 4) == i


def test_little_endian_byte_order():
    m = make_mem()
    m.write_u32(0, 0x01020304)
    assert m.read_bytes(0, 4) == bytes([0x04, 0x03, 0x02, 0x01])


def test_wstrb_all_bytes():
    m = make_mem()
    m.write_u32(0, 0xFFFFFFFF, wstrb=0xF)
    assert m.read_u32(0) == 0xFFFFFFFF


def test_wstrb_byte0_only():
    m = make_mem()
    m.write_u32(0, 0xFFFFFFFF, wstrb=0x1)
    assert m.read_u32(0) == 0x000000FF


def test_wstrb_partial_write():
    m = make_mem()
    m.write_u32(0, 0xAABBCCDD)
    m.write_u32(0, 0x0000FE00, wstrb=0b0010)
    word = m.read_u32(0)
    assert (word >> 8) & 0xFF == 0xFE
    assert word & 0xFF == 0xDD
    assert (word >> 16) & 0xFF == 0xBB
    assert (word >> 24) & 0xFF == 0xAA


def test_wstrb_zero_no_write():
    m = make_mem()
    m.write_u32(0, 0x12345678)
    m.write_u32(0, 0xFFFFFFFF, wstrb=0x0)
    assert m.read_u32(0) == 0x12345678


def test_out_of_range_read_raises():
    m = make_mem(size=16)
    with pytest.raises(MemoryRangeError):
        m.read_u32(16)


def test_out_of_range_write_raises():
    m = make_mem(size=16)
    with pytest.raises(MemoryRangeError):
        m.write_u32(16, 0xDEAD)


def test_last_word_ok():
    m = make_mem(size=16)
    m.write_u32(12, 0xCAFEBABE)
    assert m.read_u32(12) == 0xCAFEBABE


def test_reset_clears():
    m = make_mem()
    m.write_u32(0, 0xFFFFFFFF)
    m.reset()
    assert m.read_u32(0) == 0x00000000