"""
MockByteStream — in-memory stream for unit testing without a real UART.
"""

from __future__ import annotations

from collections import deque


class MockByteStream:
    def __init__(self, rx: bytes = b"") -> None:
        self._rx: deque[int] = deque(rx)
        self._tx = bytearray()

    def read_byte(self) -> int:
        if not self._rx:
            raise RuntimeError("MockByteStream RX buffer empty")
        return self._rx.popleft()

    def write_bytes(self, data: bytes) -> None:
        self._tx.extend(data)

    def push_rx(self, data: bytes) -> None:
        self._rx.extend(data)

    def take_tx(self) -> bytes:
        data = bytes(self._tx)
        self._tx.clear()
        return data

    def rx_empty(self) -> bool:
        return len(self._rx) == 0