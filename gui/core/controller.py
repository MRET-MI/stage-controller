from __future__ import annotations

from dataclasses import dataclass

from .commands import (
    gpio_read,
    gpio_set,
    home,
    home_physical,
    jog_start,
    jog_start_physical,
    jog_stop,
    jog_stop_physical,
    move_absolute,
    move_physical_absolute,
    move_physical_relative,
    move_relative,
    physical_speed_query,
    physical_speed_set,
    position_physical_query,
    position_query,
    pulse_speed_query,
    pulse_speed_set,
    register_read,
    register_read_all,
    register_reapply,
    register_save,
    register_set,
    route,
)
from comm.transport import StageTransport


@dataclass
class StageController:
    """カスケード対応コントローラ。全コマンドを @<box> ルーティングで送信する。

    - 軸操作は「機番 box + ローカル軸 axis(0..2 または X/Y/Z)」で指定する。
      GUI 側はグローバル軸番号 g を box=g//3, local=g%3 に換算して渡す。
    - box=0 はマスター自機（ファーム側でローカル実行）。
    """
    transport: StageTransport

    def connect(self) -> str:
        self.transport.open()
        return "connected"

    def disconnect(self) -> None:
        self.transport.close()

    def is_connected(self) -> bool:
        return self.transport.is_open()

    # ── 内部: ルーティング送信 ───────────────────────────────────────────────────

    def _send(self, box: int, subcmd: str) -> str:
        return self.transport.send_command(route(box, subcmd))

    # ── Speed（対象機の一時状態。移動コマンドにも MS/mS が埋め込まれる） ──────────

    def set_pulse_speed(self, freq: int, box: int = 0) -> str:
        return self._send(box, pulse_speed_set(freq))

    def get_pulse_speed(self, box: int = 0) -> str:
        return self._send(box, pulse_speed_query())

    def set_physical_speed(self, speed: float, box: int = 0) -> str:
        return self._send(box, physical_speed_set(speed))

    def get_physical_speed(self, box: int = 0) -> str:
        return self._send(box, physical_speed_query())

    # ── Pulse-based moves（box + ローカル軸） ────────────────────────────────────

    def move_relative(self, box: int, axis: str, pulses: int, speed_pps: int, *, wait: bool = False) -> str:
        return self._send(box, move_relative(axis, pulses, speed_pps, wait=wait))

    def move_absolute(self, box: int, axis: str, pulses: int, speed_pps: int, *, wait: bool = False) -> str:
        return self._send(box, move_absolute(axis, pulses, speed_pps, wait=wait))

    def get_position(self, box: int, axis: str) -> str:
        return self._send(box, position_query(axis))

    def home(self, box: int, axis: str, *, wait: bool = False) -> str:
        return self._send(box, home(axis, wait=wait))

    def jog_start(self, box: int, axis: str, positive: bool) -> str:
        return self._send(box, jog_start(axis, positive))

    def jog_stop(self, box: int, axis: str) -> str:
        return self._send(box, jog_stop(axis))

    # ── Physical-unit moves ──────────────────────────────────────────────────────

    def move_physical_relative(self, box: int, axis: str, distance: float, speed: float, *, wait: bool = False) -> str:
        return self._send(box, move_physical_relative(axis, distance, speed, wait=wait))

    def move_physical_absolute(self, box: int, axis: str, position: float, speed: float, *, wait: bool = False) -> str:
        return self._send(box, move_physical_absolute(axis, position, speed, wait=wait))

    def get_physical_position(self, box: int, axis: str) -> str:
        return self._send(box, position_physical_query(axis))

    def home_physical(self, box: int, axis: str, *, wait: bool = False) -> str:
        return self._send(box, home_physical(axis, wait=wait))

    def jog_start_physical(self, box: int, axis: str, positive: bool) -> str:
        return self._send(box, jog_start_physical(axis, positive))

    def jog_stop_physical(self, box: int, axis: str) -> str:
        return self._send(box, jog_stop_physical(axis))

    # ── Register（対象機へルーティング） ─────────────────────────────────────────

    def save_registers(self, box: int = 0) -> str:
        return self._send(box, register_save())

    def reapply(self, box: int = 0) -> str:
        return self._send(box, register_reapply())

    def read_all_registers(self, box: int = 0) -> str:
        return self._send(box, register_read_all())

    def set_register(self, address: int, value: int | float, box: int = 0) -> str:
        return self._send(box, register_set(address, value))

    def get_register(self, address: int, box: int = 0) -> str:
        return self._send(box, register_read(address))

    # ── GPIO 出力（マスター機のみ。ルーティングせず自機ローカル実行） ────────────

    def gpio_set(self, n: int, level: int) -> str:
        return self.transport.send_command(gpio_set(n, level))

    def gpio_read(self, n: int) -> str:
        return self.transport.send_command(gpio_read(n))
