"""Small mock byte stream for protocol testing"""

from __future__ import annotations

from collections import deque


class MockByteStream:
    def __init__(self) -> None:
        self.rx = deque()
        self.tx = bytearray()

    def read_byte(self) -> int:
        if not self.rx:
            raise TimeoutError("Mock stream timeout")
        return self.rx.popleft()

    def write_bytes(self, data: bytes) -> None:
        self.tx.extend(data)

    def push_rx(self, data: bytes) -> None:
        self.rx.extend(data)

    def pop_tx(self) -> bytes:
        data = bytes(self.tx)
        self.tx.clear()
        return data