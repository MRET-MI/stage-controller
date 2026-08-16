"""
settings_dialog.py
ステージ軸設定ダイアログ。
各軸の μm/pulse 係数・速度上限・ソフトリミット・ホームオフセットを入力し、
MCU への書き込み（RS コマンドでフラッシュ保存）と読み込みを行う。
"""
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from core.stage_config import (
    AXES,
    MICROSTEP_OPTIONS,
    MOTOR_MODELS,
    AxisConfig,
    BoxConfig,
    GpioConfig,
    StageConfig,
)

if TYPE_CHECKING:
    from core.controller import StageController


class _AxisWidget(QWidget):
    """1軸分の設定フォーム"""

    def __init__(self, cfg: AxisConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._build(cfg)

    def _dspin(
        self,
        lo: float,
        hi: float,
        step: float,
        suffix: str,
        value: float,
        decimals: int = 4,
    ) -> QDoubleSpinBox:
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setSingleStep(step)
        w.setDecimals(decimals)
        w.setSuffix(suffix)
        w.setValue(value)
        return w

    def _build(self, cfg: AxisConfig) -> None:
        form = QFormLayout(self)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        form.setContentsMargins(12, 12, 12, 12)
        form.setVerticalSpacing(8)

        # 1フルステップ移動量 [μm/fullstep]
        self.um_per_fullstep = self._dspin(1e-4, 1e6, 0.1, " μm/full", cfg.um_per_fullstep, 4)
        form.addRow("1フルステップ移動量", self.um_per_fullstep)

        # 分解能（microstep）
        self.microstep = QComboBox()
        for ms in MICROSTEP_OPTIONS:
            self.microstep.addItem(f"1/{ms}" if ms != 1 else "フルステップ", ms)
        idx = self.microstep.findData(cfg.microstep)
        self.microstep.setCurrentIndex(idx if idx >= 0 else 0)
        form.addRow("分解能", self.microstep)

        # → um/pulse 算出値（読み取り専用表示）
        self.upp_label = QLabel()
        self.um_per_fullstep.valueChanged.connect(self._update_upp)
        self.microstep.currentIndexChanged.connect(self._update_upp)
        form.addRow("→ μm/pulse", self.upp_label)

        # モーター型番（MOT_SEL）
        self.motor_model = QComboBox()
        for name, val in MOTOR_MODELS.items():
            self.motor_model.addItem(name, val)
        midx = self.motor_model.findData(cfg.motor_model)
        self.motor_model.setCurrentIndex(midx if midx >= 0 else 0)
        form.addRow("モーター型番", self.motor_model)

        # 初速度・加速時間
        self.start_speed = self._dspin(0.0, 500_000.0, 100.0, " μm/s", cfg.start_speed_um_s, 1)
        form.addRow("初速度", self.start_speed)
        self.accel_time = QSpinBox()
        self.accel_time.setRange(1, 60_000)
        self.accel_time.setSingleStep(10)
        self.accel_time.setSuffix(" ms")
        self.accel_time.setValue(cfg.accel_time_ms)
        form.addRow("加速時間", self.accel_time)

        # 速度上限
        self.max_speed = self._dspin(1.0, 500_000.0, 500.0, " μm/s", cfg.max_speed_um_s, 1)
        form.addRow("最大速度", self.max_speed)

        # ソフトリミット
        self.limit_cw  = self._dspin(-1e7, 1e7, 1000.0, " μm", cfg.limit_cw_um,  1)
        self.limit_ccw = self._dspin(-1e7, 1e7, 1000.0, " μm", cfg.limit_ccw_um, 1)
        form.addRow("CW リミット",  self.limit_cw)
        form.addRow("CCW リミット", self.limit_ccw)

        # ホームオフセット
        self.home_offset = self._dspin(-1e7, 1e7, 100.0, " μm", cfg.home_offset_um, 1)
        form.addRow("ホームオフセット", self.home_offset)

        self._update_upp()

    def _update_upp(self) -> None:
        """1フルステップ移動量と microstep から um/pulse を算出表示する"""
        upf = self.um_per_fullstep.value()
        ms  = self.microstep.currentData()
        upp = upf / ms if ms else 0.0
        self.upp_label.setText(f"{upp:.6g} μm/pulse")

    def to_axis_config(self) -> AxisConfig:
        """ウィジェットの現在値から AxisConfig を生成する"""
        return AxisConfig(
            um_per_fullstep  = self.um_per_fullstep.value(),
            microstep        = self.microstep.currentData(),
            start_speed_um_s = self.start_speed.value(),
            accel_time_ms    = self.accel_time.value(),
            motor_model      = self.motor_model.currentData(),
            max_speed_um_s   = self.max_speed.value(),
            limit_cw_um      = self.limit_cw.value(),
            limit_ccw_um     = self.limit_ccw.value(),
            home_offset_um   = self.home_offset.value(),
        )

    def load_from_axis_config(self, cfg: AxisConfig) -> None:
        """AxisConfig の値でウィジェットを更新する"""
        self.um_per_fullstep.setValue(cfg.um_per_fullstep)
        idx = self.microstep.findData(cfg.microstep)
        self.microstep.setCurrentIndex(idx if idx >= 0 else 0)
        midx = self.motor_model.findData(cfg.motor_model)
        self.motor_model.setCurrentIndex(midx if midx >= 0 else 0)
        self.start_speed.setValue(cfg.start_speed_um_s)
        self.accel_time.setValue(cfg.accel_time_ms)
        self.max_speed.setValue(cfg.max_speed_um_s)
        self.limit_cw.setValue(cfg.limit_cw_um)
        self.limit_ccw.setValue(cfg.limit_ccw_um)
        self.home_offset.setValue(cfg.home_offset_um)


class _GpioWidget(QWidget):
    """汎用 GPIO 出力(PE9..PE12) の設定フォーム（ラベル名2個・極性4個）"""

    _PIN_NAMES = ("PE9", "PE10", "PE11", "PE12")

    def __init__(self, cfg: GpioConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        form = QFormLayout(self)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        form.setContentsMargins(12, 12, 12, 12)
        form.setVerticalSpacing(8)

        self.label_edits: list[QLineEdit] = []
        self.polarity_combos: list[QComboBox] = []

        for pair in range(2):
            edit = QLineEdit(cfg.labels[pair] if pair < len(cfg.labels) else f"GPIO {pair + 1}")
            self.label_edits.append(edit)
            form.addRow(f"ラベル名 {pair + 1}（{self._PIN_NAMES[pair*2]}/{self._PIN_NAMES[pair*2+1]}）", edit)
            for k in range(2):
                n = pair * 2 + k
                combo = QComboBox()
                combo.addItem("High = ON", True)
                combo.addItem("Low = ON",  False)
                idx = 0 if (n < len(cfg.active_high) and cfg.active_high[n]) else 1
                combo.setCurrentIndex(idx)
                self.polarity_combos.append(combo)
                form.addRow(f"{self._PIN_NAMES[n]} 極性", combo)

    def to_gpio_config(self) -> GpioConfig:
        return GpioConfig(
            labels      = [e.text() for e in self.label_edits],
            active_high = [bool(c.currentData()) for c in self.polarity_combos],
        )


class _BoxWidget(QWidget):
    """1機（3軸＋役割）の設定フォーム。軸は内側タブ(X/Y/Z)。"""

    def __init__(self, box: BoxConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        outer = QVBoxLayout(self)

        # 役割（機番は並び順に固定＝表示のみ, is_master のみ編集可）
        role = QHBoxLayout()
        role.addWidget(QLabel(f"機番 (BOX_NO): {box.box_no}"))
        self.is_master = QCheckBox("マスター機（PCとUSB通信する機体）")
        self.is_master.setChecked(box.is_master)
        role.addWidget(self.is_master)
        role.addStretch()
        outer.addLayout(role)

        # 軸タブ
        self.axis_widgets: dict[str, _AxisWidget] = {}
        tabs = QTabWidget()
        for ax in AXES:
            w = _AxisWidget(box.axes[ax])
            self.axis_widgets[ax] = w
            tabs.addTab(w, f"軸 {ax}")
        outer.addWidget(tabs)

    def collect_into(self, box: BoxConfig) -> None:
        box.is_master = self.is_master.isChecked()
        for ax, w in self.axis_widgets.items():
            box.axes[ax] = w.to_axis_config()

    def load_axes(self, box: BoxConfig) -> None:
        for ax, w in self.axis_widgets.items():
            w.load_from_axis_config(box.axes[ax])

    def load_role(self, box: BoxConfig) -> None:
        self.is_master.setChecked(box.is_master)


class StageSettingsDialog(QDialog):
    """ステージ設定ダイアログ（複数機カスケード対応）"""

    def __init__(
        self,
        config: StageConfig,
        config_path: Path,
        controller: StageController | None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("ステージ設定")
        self.setMinimumWidth(520)
        self._config = config
        self._config_path = config_path
        self._controller = controller
        self._box_widgets: list[_BoxWidget] = []
        self._build_ui()

    def _build_ui(self) -> None:
        self._root = QVBoxLayout(self)

        # 接続台数
        top = QHBoxLayout()
        top.addWidget(QLabel("接続台数（カスケード）"))
        self._num_boxes = QSpinBox()
        self._num_boxes.setRange(1, 4)
        self._num_boxes.setValue(self._config.num_boxes)
        self._num_boxes.setSuffix(" 台")
        self._num_boxes.valueChanged.connect(self._on_num_boxes_changed)
        top.addWidget(self._num_boxes)
        top.addStretch()
        self._root.addLayout(top)

        # 機体タブ（＋ GPIO タブ）
        self._tabs = QTabWidget()
        self._root.addWidget(self._tabs)
        self._rebuild_box_tabs()

        # MCU 操作ボタン（選択中の機体を対象）
        mcu_row = QHBoxLayout()
        self._write_btn = QPushButton("この機に書込・保存")
        self._read_btn  = QPushButton("この機から読込")
        self._write_btn.clicked.connect(self._write_current_box)
        self._read_btn.clicked.connect(self._read_current_box)
        mcu_row.addWidget(self._write_btn)
        mcu_row.addWidget(self._read_btn)
        mcu_row.addStretch()
        self._root.addLayout(mcu_row)

        if self._controller is None:
            self._write_btn.setEnabled(False)
            self._read_btn.setEnabled(False)

        # OK / キャンセル
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self._accept)
        buttons.rejected.connect(self.reject)
        self._root.addWidget(buttons)

    # ── タブ再構築（台数変更時） ──────────────────────────────────────────────

    def _rebuild_box_tabs(self) -> None:
        self._tabs.clear()
        self._box_widgets = []
        for i, box in enumerate(self._config.boxes):
            w = _BoxWidget(box)
            self._box_widgets.append(w)
            role = "マスター" if box.is_master else "スレーブ"
            self._tabs.addTab(w, f"機{box.box_no}（{role}）")
        self._gpio_widget = _GpioWidget(self._config.gpio)
        self._tabs.addTab(self._gpio_widget, "GPIO")

    def _on_num_boxes_changed(self, n: int) -> None:
        # 既存の編集値を退避 → 台数変更 → タブ再構築
        self._collect()
        self._config.set_num_boxes(n)
        self._rebuild_box_tabs()

    # ── 内部処理 ──────────────────────────────────────────────────────────────

    def _collect(self) -> None:
        """ウィジェットの値を _config に反映する（GPIO 含む）"""
        for i, w in enumerate(self._box_widgets):
            if i < len(self._config.boxes):
                w.collect_into(self._config.boxes[i])
        self._config.gpio = self._gpio_widget.to_gpio_config()

    def _accept(self) -> None:
        self._collect()
        try:
            self._config.save(self._config_path)
        except Exception as exc:
            QMessageBox.warning(self, "保存エラー", f"設定ファイルの保存に失敗しました:\n{exc}")
        self.accept()

    def _current_box_idx(self) -> int:
        """GPIO タブ以外で選択中の機体インデックス。GPIO タブ時は 0。"""
        idx = self._tabs.currentIndex()
        return idx if idx < len(self._box_widgets) else 0

    def _write_current_box(self) -> None:
        self._collect()
        idx = self._current_box_idx()
        try:
            self._config.write_box_to_mcu(self._controller, idx)
            box_no = self._config.boxes[idx].box_no
            QMessageBox.information(self, "完了", f"機{box_no} へ書き込み、フラッシュへ保存しました。")
        except Exception as exc:
            QMessageBox.critical(self, "エラー", f"MCU への書き込みに失敗しました:\n{exc}")

    def _read_current_box(self) -> None:
        idx = self._current_box_idx()
        try:
            self._config.read_box_from_mcu(self._controller, idx)
        except Exception as exc:
            QMessageBox.critical(self, "エラー", f"MCU からの読み込みに失敗しました:\n{exc}")
            return
        self._box_widgets[idx].load_axes(self._config.boxes[idx])
        self._box_widgets[idx].load_role(self._config.boxes[idx])   # マスターチェックも更新
        box_no = self._config.boxes[idx].box_no
        QMessageBox.information(self, "完了", f"機{box_no} から読み込みました。")
