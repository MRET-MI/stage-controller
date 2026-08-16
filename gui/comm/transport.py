from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


class TransportError(RuntimeError):
    pass


class StageTransport(ABC):
    @abstractmethod
    def open(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def is_open(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def send_command(self, command: str) -> str:
        raise NotImplementedError


@dataclass
class MockTransport(StageTransport):
    opened: bool = False

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.opened = False

    def is_open(self) -> bool:
        return self.opened

    def send_command(self, command: str) -> str:
        if not self.opened:
            raise TransportError("Mock transport is not connected.")
        return f"OK {command}"


@dataclass
class SerialTransport(StageTransport):
    port: str
    baudrate: int = 115200
    timeout: float = 1.0
    line_ending: bytes = b"\n"

    def __post_init__(self) -> None:
        self._serial = None

    def open(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise TransportError("pyserial is not installed.") from exc

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
        except serial.SerialException as exc:
            raise TransportError(str(exc)) from exc

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    def send_command(self, command: str) -> str:
        if not self.is_open():
            raise TransportError("Serial port is not connected.")

        payload = command.encode("ascii") + self.line_ending
        try:
            self._serial.write(payload)
            self._serial.flush()
            response = self._serial.readline().decode("ascii", errors="replace").strip()
        except Exception as exc:
            raise TransportError(str(exc)) from exc

        return response or "NO RESPONSE"
