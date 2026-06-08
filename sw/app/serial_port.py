"""SerialByteStream wrapper over pyserial"""

from __future__ import annotations

from typing import List

import serial
from serial.tools import list_ports



def list_serial_ports() -> List[str]:
    return [port.device for port in list_ports.comports()]


class SerialByteStream:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.2) -> None:
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

    @property
    def port(self) -> str:
        return str(self._ser.port)

    @property
    def baudrate(self) -> int:
        return int(self._ser.baudrate)

    @property
    def is_open(self) -> bool:
        return bool(self._ser.is_open)

    def read_byte(self) -> int:
        data = self._ser.read(1)
        if not data:
            raise TimeoutError("Serial port read timeout")
        return data[0]

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
        return f"SerialByteStream(port={self._ser.port!r}, baud={self._ser.baudrate})"