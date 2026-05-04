"""
MemoryModel — byte-addressable little-endian 32-bit memory.

Mirrors the Verilog testbench pc_mem[] behaviour:
- word-aligned 32-bit reads/writes
- byte-granular writes via WSTRB mask (bit i → byte i of the word)
- out-of-range → raises MemoryRangeError (caller maps to BAD_ADDR status)
"""

from __future__ import annotations

from pathlib import Path


class MemoryError(Exception):
    pass


class MemoryRangeError(MemoryError):
    pass


class MemoryModel:
    def __init__(self, size_bytes: int = 4096 * 4) -> None:
        if size_bytes % 4 != 0:
            raise ValueError("size_bytes must be a multiple of 4")
        self.size_bytes = size_bytes
        self._mem = bytearray(size_bytes)

    def reset(self, fill: int = 0x00) -> None:
        self._mem[:] = bytes([fill & 0xFF]) * self.size_bytes

    def __len__(self) -> int:
        return self.size_bytes

    def _check(self, addr: int, size: int = 4) -> None:
        if addr < 0 or (addr + size) > self.size_bytes:
            raise MemoryRangeError(
                f"Address 0x{addr:08X}+{size} out of range "
                f"[0x00000000, 0x{self.size_bytes - 1:08X}]"
            )

    def read_bytes(self, addr: int, count: int) -> bytes:
        self._check(addr, count)
        return bytes(self._mem[addr:addr + count])

    def write_bytes(self, addr: int, data: bytes | bytearray) -> None:
        self._check(addr, len(data))
        self._mem[addr:addr + len(data)] = data

    def read_u32(self, addr: int) -> int:
        self._check(addr, 4)
        return int.from_bytes(self._mem[addr:addr + 4], byteorder="little")

    def write_u32(self, addr: int, value: int, wstrb: int = 0xF) -> None:
        """Write up to 4 bytes of value to addr, gated by wstrb bitmask."""
        self._check(addr, 4)
        raw = (value & 0xFFFF_FFFF).to_bytes(4, byteorder="little")
        for i in range(4):
            if (wstrb >> i) & 1:
                self._mem[addr + i] = raw[i]

    def load_hex_words(self, path: str | Path, base_addr: int = 0) -> int:
        """Load a plain hex file (one 32-bit word per line, no addresses)."""
        path = Path(path)
        addr = base_addr
        count = 0
        with path.open("r", encoding="utf-8") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith(("//", "#", ";")):
                    continue
                word = int(s, 16) & 0xFFFF_FFFF
                self.write_u32(addr, word, 0xF)
                addr += 4
                count += 1
        return count

    def dump_words(self, addr: int, count: int) -> list[int]:
        return [self.read_u32(addr + i * 4) for i in range(count)]

    def dump_hex(self, addr: int, count: int) -> str:
        words = self.dump_words(addr, count)
        lines = []
        for i, w in enumerate(words):
            lines.append(f"  0x{addr + i*4:08X}: 0x{w:08X}")
        return "\n".join(lines)