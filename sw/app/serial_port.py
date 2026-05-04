"""
SerialByteStream — wraps pyserial into the ByteStream interface required
by UartMemoryAgent.
"""

from __future__ import annotations

import serial


class SerialByteStream:
    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 5.0,
    ) -> None:
        self._ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        if not self._ser.is_open:
            self._ser.open()

    def read_byte(self) -> int:
        b = self._ser.read(1)
        if not b:
            raise TimeoutError("Serial port read timeout")
        return b[0]

    def write_bytes(self, data: bytes) -> None:
        self._ser.write(data)
        self._ser.flush()

    def __enter__(self) -> "SerialByteStream":
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    def __repr__(self) -> str:
        return (
            f"SerialByteStream(port={self._ser.port!r}, "
            f"baud={self._ser.baudrate})"
        )