from __future__ import annotations

import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)

from core.commands import axis_no, route
from core.controller import StageController
from core.stage_config import StageConfig
from comm.transport import MockTransport, SerialTransport, TransportError
from ui.settings_dialog import StageSettingsDialog


@dataclass
class _AxisRow:
    """1軸分のウィジェット群"""
    jog_minus:       QPushButton
    jog_plus:        QPushButton
    dist_spin:       QDoubleSpinBox   # 移動量 [μm]（相対）
    speed_spin:      QDoubleSpinBox   # 移動速度 [μm/s]
    move_button:     QPushButton      # 相対移動
    target_spin:     QDoubleSpinBox   # 目標位置 [μm]（未編集時は現在位置を表示）
    abs_move_button: QPushButton      # 絶対移動
    home_button:     QPushButton
    stop_button:     QPushButton

    def all_widgets(self) -> list[QWidget]:
        return [
            self.jog_minus, self.jog_plus,
            self.dist_spin, self.speed_spin, self.move_button,
            self.target_spin, self.abs_move_button,
            self.home_button, self.stop_button,
        ]


def _make_axis_row() -> _AxisRow:
    minus = QPushButton("−")
    plus_ = QPushButton("＋")
    minus.setAutoRepeat(False)
    plus_.setAutoRepeat(False)

    dist = QDoubleSpinBox()
    dist.setRange(-100_000.0, 100_000.0)
    dist.setSingleStep(100.0)
    dist.setDecimals(2)
    dist.setValue(1_000.0)
    dist.setSuffix(" μm")

    speed = QDoubleSpinBox()
    speed.setRange(1.0, 500_000.0)
    speed.setSingleStep(500.0)
    speed.setDecimals(1)
    speed.setValue(5_000.0)
    speed.setSuffix(" μm/s")

    # 目標位置 [μm]: 未編集時は現在位置を表示（ポーリングで追従）、編集して「絶対移動」でその位置へ
    target = QDoubleSpinBox()
    target.setRange(-1_000_000.0, 1_000_000.0)
    target.setSingleStep(100.0)
    target.setDecimals(2)
    target.setValue(0.0)
    target.setSuffix(" μm")
    target.setObjectName("targetSpin")

    stop = QPushButton("停止")
    stop.setObjectName("stopButton")

    return _AxisRow(
        jog_minus=minus,
        jog_plus=plus_,
        dist_spin=dist,
        speed_spin=speed,
        move_button=QPushButton("移動"),
        target_spin=target,
        abs_move_button=QPushButton("絶対移動"),
        home_button=QPushButton("原点"),
        stop_button=stop,
    )


class MainWindow(QMainWindow):
    def __init__(self, config_path: Path) -> None:
        super().__init__()
        self.setWindowTitle("自動ステージコントローラ")
        self.resize(1080, 580)
        self.controller:   StageController | None = None
        self._config_path: Path = config_path
        self._config:      StageConfig = StageConfig.load_or_default(config_path)
        self._naxes:       int = self._config.num_axes   # グローバル軸数（3×台数）

        # ── 位置ポーリング用タイマー（接続中のみ稼働） ──────────────────────────
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(300)   # 300 ms
        self._poll_timer.timeout.connect(self._poll_positions)

        # ── 通信 ──────────────────────────────────────────────────────────────
        self.port_combo        = QComboBox()
        self.mock_check        = QCheckBox("シミュレーション")
        self.mock_check.setChecked(True)
        self.connect_button    = QPushButton("接続")
        self.disconnect_button = QPushButton("切断")
        self.refresh_button    = QPushButton("更新")
        self.settings_button   = QPushButton("設定…")
        self.disconnect_button.setEnabled(False)

        # ── 各軸ウィジェット（グローバル軸 0..naxes-1, 添字=グローバル軸番号） ──────
        self._rows: list[_AxisRow] = [_make_axis_row() for _ in range(self._naxes)]

        # ── 汎用 GPIO 出力（PE9..PE12, トグル） ───────────────────────────────
        self._gpio_names = ("PE9", "PE10", "PE11", "PE12")
        self._gpio_buttons: list[QPushButton] = []
        for name in self._gpio_names:
            b = QPushButton(name)
            b.setCheckable(True)
            b.setObjectName("gpioToggle")
            self._gpio_buttons.append(b)
        self._gpio_pair_labels = [QLabel(), QLabel()]
        for lbl in self._gpio_pair_labels:
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # ── ジョグ速度（全軸共通）[μm/s] ──────────────────────────────────────
        self.jog_speed_spin = QDoubleSpinBox()
        self.jog_speed_spin.setRange(1.0, 500_000.0)
        self.jog_speed_spin.setSingleStep(500.0)
        self.jog_speed_spin.setDecimals(1)
        self.jog_speed_spin.setValue(1_000.0)
        self.jog_speed_spin.setSuffix(" μm/s")

        # ── 直接コマンド ───────────────────────────────────────────────────────
        self.command_edit = QPlainTextEdit()
        self.command_edit.setPlaceholderText("例: mS5000;m0R1000")
        self.command_edit.setMaximumHeight(68)
        self.send_button = QPushButton("送信")

        # ── ログ ──────────────────────────────────────────────────────────────
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)

        self._build_ui()
        self._connect_signals()
        self.refresh_ports()
        self._set_controls_enabled(False)

    # ── UI 構築 ───────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(10)
        root.addWidget(self._build_connection_box())
        root.addWidget(self._build_stage_box())
        root.addWidget(self._build_gpio_box())
        root.addWidget(self._build_command_box())
        root.addWidget(self._build_log_box(), stretch=1)
        self.setCentralWidget(central)
        self._apply_style()

    def _build_connection_box(self) -> QGroupBox:
        box = QGroupBox("通信")
        lay = QGridLayout(box)
        lay.addWidget(QLabel("ポート"),       0, 0)
        lay.addWidget(self.port_combo,        0, 1)
        lay.addWidget(self.mock_check,        0, 2)
        lay.addWidget(self.refresh_button,    0, 3)
        lay.addWidget(self.connect_button,    0, 4)
        lay.addWidget(self.disconnect_button, 0, 5)
        lay.addWidget(self.settings_button,   0, 6)
        return box

    def _build_stage_box(self) -> QGroupBox:
        box = QGroupBox("ステージ操作")
        outer = QVBoxLayout(box)
        outer.setSpacing(8)

        jog_row = QHBoxLayout()
        jog_row.addWidget(QLabel("ジョグ速度（全軸共通）"))
        jog_row.addWidget(self.jog_speed_spin)
        jog_row.addStretch()
        outer.addLayout(jog_row)

        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        HEADERS = [
            ("軸",            0),
            ("ジョグ −",      1),
            ("ジョグ ＋",     2),
            ("移動量 [μm]",   3),
            ("速度 [μm/s]",   4),
            ("移動",          5),
            ("目標位置 [μm]", 6),
            ("絶対移動",      7),
            ("原点",          8),
            ("停止",          9),
        ]
        for text, col in HEADERS:
            lbl = QLabel(text)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(lbl, 0, col)

        for g in range(self._naxes):
            r        = self._rows[g]
            box_no, local = self._config.axis_location(g)
            row_idx  = g + 1
            # 複数機ならラベルは「機番:軸」（例 0:X）、単機なら軸名のみ
            name     = f"{box_no}:{local}" if self._config.num_boxes > 1 else local
            ax_lbl   = QLabel(name)
            ax_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(ax_lbl,            row_idx, 0)
            grid.addWidget(r.jog_minus,       row_idx, 1)
            grid.addWidget(r.jog_plus,        row_idx, 2)
            grid.addWidget(r.dist_spin,       row_idx, 3)
            grid.addWidget(r.speed_spin,      row_idx, 4)
            grid.addWidget(r.move_button,     row_idx, 5)
            grid.addWidget(r.target_spin,     row_idx, 6)
            grid.addWidget(r.abs_move_button, row_idx, 7)
            grid.addWidget(r.home_button,     row_idx, 8)
            grid.addWidget(r.stop_button,     row_idx, 9)

        grid.setColumnStretch(3, 2)
        grid.setColumnStretch(4, 2)
        grid.setColumnStretch(6, 2)
        outer.addLayout(grid)
        return box

    def _build_gpio_box(self) -> QGroupBox:
        box = QGroupBox("GPIO 出力")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        # 2ペア × [左ボタン][ラベル][右ボタン]  (ペア0: PE9/PE10, ペア1: PE11/PE12)
        for pair in range(2):
            left  = self._gpio_buttons[pair * 2]
            right = self._gpio_buttons[pair * 2 + 1]
            grid.addWidget(left,                       pair, 0)
            grid.addWidget(self._gpio_pair_labels[pair], pair, 1)
            grid.addWidget(right,                      pair, 2)
        grid.setColumnStretch(1, 1)
        self._refresh_gpio_labels()
        return box

    def _build_command_box(self) -> QGroupBox:
        box = QGroupBox("直接コマンド")
        lay = QVBoxLayout(box)
        lay.addWidget(self.command_edit)
        lay.addWidget(self.send_button, alignment=Qt.AlignmentFlag.AlignRight)
        return box

    def _build_log_box(self) -> QGroupBox:
        box = QGroupBox("ログ")
        lay = QVBoxLayout(box)
        lay.addWidget(self.log)
        return box

    def _apply_style(self) -> None:
        self.setStyleSheet("""
            QMainWindow { background: #f5f7fb; }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #d9dee8;
                border-radius: 6px;
                margin-top: 12px;
                padding: 10px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                min-height: 28px;
                min-width: 56px;
                padding: 2px 6px;
            }
            QPushButton#stopButton {
                background: #b42318;
                color: white;
                font-weight: 700;
            }
            QPushButton#gpioToggle {
                min-width: 72px;
            }
            QPushButton#gpioToggle:checked {
                background: #12b76a;
                color: white;
                font-weight: 700;
            }
            QDoubleSpinBox#targetSpin {
                font-family: Consolas, monospace;
                font-weight: 600;
                background: #eef2f9;
            }
            QPlainTextEdit {
                font-family: Consolas, monospace;
                font-size: 10pt;
            }
            QLabel {
                font-size: 9pt;
            }
        """)

    # ── シグナル接続 ──────────────────────────────────────────────────────────

    def _connect_signals(self) -> None:
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.connect_stage)
        self.disconnect_button.clicked.connect(self.disconnect_stage)
        self.settings_button.clicked.connect(self.open_settings)
        self.send_button.clicked.connect(self.send_manual_command)

        for n, b in enumerate(self._gpio_buttons):
            b.clicked.connect(lambda checked, n=n: self._toggle_gpio(n, checked))

        for g, r in enumerate(self._rows):
            r.move_button.clicked.connect(lambda _, ga=g: self._execute_move(ga))
            r.abs_move_button.clicked.connect(lambda _, ga=g: self._execute_move_absolute(ga))
            r.home_button.clicked.connect(lambda _, ga=g: self._home(ga))
            r.stop_button.clicked.connect(lambda _, ga=g: self._jog_stop(ga))
            r.jog_minus.pressed.connect(lambda ga=g: self._jog_start(ga, positive=False))
            r.jog_minus.released.connect(lambda ga=g: self._jog_stop(ga))
            r.jog_plus.pressed.connect(lambda ga=g: self._jog_start(ga, positive=True))
            r.jog_plus.released.connect(lambda ga=g: self._jog_stop(ga))

    # ── 接続 ──────────────────────────────────────────────────────────────────

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [port.portName() for port in QSerialPortInfo.availablePorts()]
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)
        self._log(f"検出ポート: {', '.join(ports) if ports else 'なし'}")

    def connect_stage(self) -> None:
        try:
            if self.mock_check.isChecked():
                transport = MockTransport()
            else:
                port = self.port_combo.currentText().strip()
                if not port:
                    QMessageBox.warning(self, "接続できません", "シリアルポートを選択してください。")
                    return
                transport = SerialTransport(port=port)
            self.controller = StageController(transport)
            self.controller.connect()
        except TransportError as exc:
            QMessageBox.critical(self, "通信エラー", str(exc))
            self._log(f"ERROR {exc}")
            return
        self._set_connected(True)
        self._log("接続しました")

    def disconnect_stage(self) -> None:
        if self.controller is not None:
            self.controller.disconnect()
        self.controller = None
        self._set_connected(False)
        self._log("切断しました")

    # ── 設定ダイアログ ────────────────────────────────────────────────────────

    def open_settings(self) -> None:
        dlg = StageSettingsDialog(
            config      = self._config,
            config_path = self._config_path,
            controller  = self.controller,
            parent      = self,
        )
        dlg.exec()
        self._refresh_gpio_labels()   # 設定変更後にラベル名を反映
        # 台数（＝表示軸数）が変わった場合は再起動で反映（軸行の再構築が必要なため）
        if self._config.num_axes != self._naxes:
            QMessageBox.information(
                self, "再起動が必要",
                f"接続台数が {self._naxes // 3} → {self._config.num_boxes} 台に変更されました。\n"
                "軸表示に反映するにはアプリを再起動してください。",
            )

    # ── 移動（μm → pulse に PC 側で換算して M-prefix コマンドを送信） ──────────

    def _execute_move(self, g: int) -> None:
        """移動量[μm] 分だけ相対移動（グローバル軸 g）"""
        r      = self._rows[g]
        cfg    = self._config.axis_config(g)
        box, local = self._config.axis_location(g)
        if cfg.um_per_pulse == 0.0:
            QMessageBox.warning(self, "設定エラー", f"軸 {box}:{local} の μm/pulse が 0 です。設定を確認してください。")
            return
        pulses = round(r.dist_spin.value()  / cfg.um_per_pulse)
        pps    = max(1, round(r.speed_spin.value() / cfg.um_per_pulse))
        self._send(lambda c, b=box, a=local, p=pulses, s=pps: c.move_relative(b, a, p, s))

    def _execute_move_absolute(self, g: int) -> None:
        """目標位置[μm] の絶対座標へ移動（グローバル軸 g）"""
        r      = self._rows[g]
        cfg    = self._config.axis_config(g)
        box, local = self._config.axis_location(g)
        if cfg.um_per_pulse == 0.0:
            QMessageBox.warning(self, "設定エラー", f"軸 {box}:{local} の μm/pulse が 0 です。設定を確認してください。")
            return
        pulses = round(r.target_spin.value() / cfg.um_per_pulse)
        pps    = max(1, round(r.speed_spin.value() / cfg.um_per_pulse))
        self._send(lambda c, b=box, a=local, p=pulses, s=pps: c.move_absolute(b, a, p, s))

    def _home(self, g: int) -> None:
        box, local = self._config.axis_location(g)
        self._send(lambda c, b=box, a=local: c.home(b, a))

    # ── ジョグ（μm/s → pps に換算して M-prefix コマンドを送信） ─────────────

    def _jog_start(self, g: int, positive: bool) -> None:
        if self.controller is None or not self.controller.is_connected():
            return
        cfg        = self._config.axis_config(g)
        box, local = self._config.axis_location(g)
        pps        = max(1, round(self.jog_speed_spin.value() / cfg.um_per_pulse)) \
                     if cfg.um_per_pulse != 0.0 else 1
        direction  = "P" if positive else "M"
        # 速度設定とジョグ開始を1行にまとめ、対象機へ @<box> ルーティング
        cmd        = route(box, f"MS{pps};M{axis_no(local)}N{direction}")
        try:
            response = self.controller.transport.send_command(cmd)
            self._log(f"< {response}")
        except TransportError as exc:
            self._log(f"ERROR {exc}")

    def _jog_stop(self, g: int) -> None:
        box, local = self._config.axis_location(g)
        self._send(lambda c, b=box, a=local: c.jog_stop(b, a))

    # ── GPIO 出力（極性を PC 側で適用し、生レベルを送信） ──────────────────────

    def _toggle_gpio(self, n: int, checked: bool) -> None:
        """トグル ON/OFF → 極性に応じた物理レベルを G<n>S<0|1> で送信"""
        gp    = self._config.gpio
        level = gp.on_level(n) if checked else gp.off_level(n)
        self._send(lambda c, n=n, lv=level: c.gpio_set(n, lv))

    def _init_gpio_outputs(self) -> None:
        """接続時に全 GPIO を OFF レベルへ初期化し、ボタン表示を同期する。"""
        if self.controller is None or not self.controller.is_connected():
            return
        gp = self._config.gpio
        for n, b in enumerate(self._gpio_buttons):
            b.blockSignals(True)
            b.setChecked(False)
            b.blockSignals(False)
            try:
                self.controller.gpio_set(n, gp.off_level(n))
            except TransportError as exc:
                self._log(f"ERROR {exc}")

    def _refresh_gpio_labels(self) -> None:
        """設定のラベル名をペアラベルへ反映する。"""
        labels = self._config.gpio.labels
        for i, lbl in enumerate(self._gpio_pair_labels):
            lbl.setText(labels[i] if i < len(labels) else f"GPIO {i + 1}")

    # ── 位置のリアルタイム表示（QTimer で 300ms ごとにポーリング） ────────────

    def _poll_positions(self) -> None:
        if self.controller is None or not self.controller.is_connected():
            return
        for g in range(self._naxes):
            box, local = self._config.axis_location(g)
            try:
                resp = self.controller.get_position(box, local)   # @<box>;M<n>P → "1234"
                um   = float(resp) * self._config.axis_config(g).um_per_pulse
                spin = self._rows[g].target_spin
                # 編集中（フォーカス中）は入力値を上書きしない
                if not spin.hasFocus():
                    spin.blockSignals(True)
                    spin.setValue(um)
                    spin.blockSignals(False)
            except (TransportError, ValueError):
                # mock 応答や解析不可・中継エラー(NG)時は最後の値を保持
                pass

    # ── コマンド直接送信 ───────────────────────────────────────────────────────

    def send_manual_command(self) -> None:
        command = self.command_edit.toPlainText().strip()
        if not command:
            return
        self._send(lambda c: c.transport.send_command(command))

    def _send(self, action) -> None:
        if self.controller is None or not self.controller.is_connected():
            QMessageBox.warning(self, "未接続", "先にコントローラへ接続してください。")
            return
        try:
            response = action(self.controller)
        except TransportError as exc:
            QMessageBox.critical(self, "通信エラー", str(exc))
            self._log(f"ERROR {exc}")
            return
        self._log(f"< {response}")

    # ── UI 状態管理 ───────────────────────────────────────────────────────────

    def _set_connected(self, connected: bool) -> None:
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
        self.refresh_button.setEnabled(not connected)
        self.port_combo.setEnabled(not connected)
        self.mock_check.setEnabled(not connected)
        self._set_controls_enabled(connected)

        # 位置ポーリングの開始/停止
        if connected:
            self._init_gpio_outputs()   # 全 GPIO を OFF に初期化
            self._poll_timer.start()
        else:
            self._poll_timer.stop()
            for r in self._rows:
                r.target_spin.blockSignals(True)
                r.target_spin.setValue(0.0)
                r.target_spin.blockSignals(False)

    def _set_controls_enabled(self, enabled: bool) -> None:
        widgets: list[QWidget] = [self.jog_speed_spin, self.command_edit, self.send_button]
        widgets.extend(self._gpio_buttons)
        for r in self._rows:
            widgets.extend(r.all_widgets())
        for w in widgets:
            w.setEnabled(enabled)

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log.appendPlainText(f"[{timestamp}] {message}")


def main(config_path: Path | None = None) -> None:
    if config_path is None:
        config_path = Path(__file__).parent.parent / "stage_config.json"
    app    = QApplication(sys.argv)
    window = MainWindow(config_path=config_path)
    window.show()
    sys.exit(app.exec())
