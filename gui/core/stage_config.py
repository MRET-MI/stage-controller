"""
stage_config.py
ステージ軸設定と MCU パラメータレジスタの読み書き。

設定の真の入力値 = 1フルステップ移動量[μm/fullstep] + 分解能(microstep)。
um/pulse はここから自動計算する（分解能との物理的整合を保証）:
    um/pulse = um_per_fullstep / microstep
    （1フルステップ = microstep パルスに分割されるため）

MCU パラメータアドレスマップ:
  addr 104-106 : COEF_NUM  [μm/pulse] ← um_per_pulse を格納
  addr 120-122 : COEF_DEN  [pulse]    ← 常に 1.0
  addr  72-74  : limit_cw   [μm]
  addr  88-90  : limit_ccw  [μm]
  addr  56-58  : home_offset[μm]
  addr 136-138 : start_pps  [pps]     ← 初速度 (μm/s ÷ um/pulse)
  addr 152-154 : accel_ms   [ms]      ← 加速時間
  addr 168-170 : resolution [reg]     ← microstep × 10
  addr 200-202 : mot_sel    [16bit]   ← モーター型番
  addr 183     : init_access_mask
  addr 184     : motor_en_mask
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from core.controller import StageController

# ── 軸定義 ────────────────────────────────────────────────────────────────────

AXES = ("X", "Y", "Z")
AXIS_IDX = {"X": 0, "Y": 1, "Z": 2}

# 分解能(microstep) → CVD RESOLUTION レジスタ値
MICROSTEP_TO_REG = {1: 10, 2: 20, 4: 40, 5: 50, 10: 100, 20: 200, 50: 500, 100: 1000}
MICROSTEP_OPTIONS = tuple(MICROSTEP_TO_REG.keys())

# モーター型番 → CVD MOT_SEL 値
MOTOR_MODELS = {
    "0.35 A": 0xFF00,
    "0.75 A": 0xFE01,
    "1.20 A": 0xFD02,
    "1.40 A": 0xFC03,
    "1.80 A": 0xFB04,
    "2.40 A": 0xFA05,
}

# ── MCU パラメータアドレス ─────────────────────────────────────────────────────

ADDR_COEFF_NUM    = (104, 105, 106)
ADDR_COEFF_DEN    = (120, 121, 122)
ADDR_LIMIT_CW     = ( 72,  73,  74)
ADDR_LIMIT_CCW    = ( 88,  89,  90)
ADDR_HOME_OFFSET  = ( 56,  57,  58)
ADDR_START        = (136, 137, 138)
ADDR_ACCEL        = (152, 153, 154)
ADDR_RESOL        = (168, 169, 170)
ADDR_MOTSEL       = (200, 201, 202)
ADDR_INIT_ACCESS  = 183
ADDR_MOTOR_EN     = 184

# カスケード接続（役割・台数）
ADDR_BOX_NO       = 1
ADDR_IS_MASTER    = 2
ADDR_NUM_BOXES    = 3


# ── データクラス ───────────────────────────────────────────────────────────────

@dataclass
class AxisConfig:
    """1軸分のステージ設定（真の入力値は um_per_fullstep + microstep）"""
    um_per_fullstep: float = 1.0       # 1フルステップ移動量 [μm/fullstep]
    microstep: int         = 100       # 分解能 (1/2/4/5/10/20/50/100)
    start_speed_um_s: float = 100.0    # 初速度 [μm/s]
    accel_time_ms: int     = 100       # 加速時間 [ms]
    motor_model: int       = 0xFE01    # モーター型番 (MOT_SEL値)
    max_speed_um_s: float  = 5000.0    # 最大速度 [μm/s]
    limit_cw_um: float     = 25000.0   # CW ソフトリミット [μm]
    limit_ccw_um: float    = 0.0       # CCW ソフトリミット [μm]
    home_offset_um: float  = 0.0       # ホームオフセット [μm]
    motor_en: bool         = True      # モーター有効
    init_access: bool      = False     # 起動時ホーミング実施

    @property
    def um_per_pulse(self) -> float:
        """[μm/pulse] = um_per_fullstep / microstep"""
        return self.um_per_fullstep / self.microstep if self.microstep != 0 else 1.0

    @property
    def resolution_reg(self) -> int:
        """CVD RESOLUTION レジスタ値 = microstep × 10"""
        return MICROSTEP_TO_REG.get(self.microstep, self.microstep * 10)


@dataclass
class GpioConfig:
    """汎用 GPIO 出力4点(PE9..PE12) の設定。
    labels : 2ペア分の表示名（PE9-PE10 / PE11-PE12 で共有）
    active_high : 各ピン(4点)の ON 極性。True=High=ON, False=Low=ON。
    極性・ラベルは PC 側でのみ使用（MCU へは書き込まない）。
    """
    labels: list[str]      = field(default_factory=lambda: ["GPIO 1", "GPIO 2"])
    active_high: list[bool] = field(default_factory=lambda: [True, True, True, True])

    def off_level(self, n: int) -> int:
        """ピン n の OFF 物理レベル（High=ON なら 0, Low=ON なら 1）"""
        return 0 if self.active_high[n] else 1

    def on_level(self, n: int) -> int:
        """ピン n の ON 物理レベル（High=ON なら 1, Low=ON なら 0）"""
        return 1 if self.active_high[n] else 0

    def logical_on(self, n: int, level: int) -> bool:
        """物理レベル(0/1) を論理 ON/OFF に変換（極性を反映）"""
        return (level == 1) if self.active_high[n] else (level == 0)


@dataclass
class BoxConfig:
    """1台（コントローラ1機＝3軸）分の設定。box_no はカスケードの機番。"""
    box_no: int    = 0
    is_master: bool = True
    axes: dict[str, AxisConfig] = field(
        default_factory=lambda: {ax: AxisConfig() for ax in AXES}
    )


@dataclass
class StageConfig:
    """複数機（カスケード）ステージ設定の集合体。boxes[i] が i 台目（3軸）。"""
    boxes: list[BoxConfig] = field(default_factory=lambda: [BoxConfig()])
    gpio: GpioConfig       = field(default_factory=GpioConfig)  # GPIO 出力はマスター機のみ

    @property
    def num_boxes(self) -> int:
        return len(self.boxes)

    @property
    def num_axes(self) -> int:
        """グローバル軸数 = 3 × 台数"""
        return 3 * len(self.boxes)

    def axis_location(self, gaxis: int) -> tuple[int, str]:
        """グローバル軸番号 → (機番 box_no, ローカル軸名 'X'/'Y'/'Z')"""
        box_idx = gaxis // 3
        local   = AXES[gaxis % 3]
        return self.boxes[box_idx].box_no, local

    def axis_config(self, gaxis: int) -> AxisConfig:
        box_idx = gaxis // 3
        return self.boxes[box_idx].axes[AXES[gaxis % 3]]

    def set_num_boxes(self, n: int) -> None:
        """台数を変更（増やすと既定設定の機を追加、減らすと末尾を削除）。機番は 0..n-1 に振り直す。"""
        n = max(1, int(n))
        while len(self.boxes) < n:
            self.boxes.append(BoxConfig(box_no=len(self.boxes), is_master=False))
        del self.boxes[n:]
        for i, box in enumerate(self.boxes):
            box.box_no = i                 # 機番＝並び順（グローバル軸換算と一致させる）
            box.is_master = (i == 0)

    # ── ファイル保存 / 読み込み ─────────────────────────────────────────────────

    def save(self, path: Path) -> None:
        data = {
            "num_boxes": self.num_boxes,
            "boxes": [asdict(b) for b in self.boxes],
            "gpio":  asdict(self.gpio),
        }
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> StageConfig:
        data = json.loads(path.read_text(encoding="utf-8"))
        gpio_data = data.get("gpio") or data.get("_gpio")
        gpio = GpioConfig(**gpio_data) if gpio_data else GpioConfig()

        if "boxes" in data:                          # 新フォーマット（複数機）
            boxes = []
            for b in data["boxes"]:
                axes = {ax: AxisConfig(**b["axes"][ax]) for ax in AXES}
                boxes.append(BoxConfig(box_no=b.get("box_no", 0),
                                       is_master=b.get("is_master", False),
                                       axes=axes))
            if not boxes:
                boxes = [BoxConfig()]
        else:                                        # 旧フォーマット（単機 X/Y/Z）→ 1機へ包む
            axd = {k: v for k, v in data.items() if k in AXES}
            axes = {ax: AxisConfig(**axd[ax]) for ax in AXES} if axd else {ax: AxisConfig() for ax in AXES}
            boxes = [BoxConfig(box_no=0, is_master=True, axes=axes)]
        return cls(boxes=boxes, gpio=gpio)

    @classmethod
    def load_or_default(cls, path: Path) -> StageConfig:
        try:
            return cls.load(path)
        except Exception:
            return cls()

    # ── MCU への書き込み / 読み込み（対象機へ @<box_no> ルーティング） ──────────────

    def write_box_to_mcu(self, controller: StageController, box_idx: int) -> None:
        """指定機の全軸パラメータ＋役割(BOX_NO/IS_MASTER/NUM_BOXES)を書き込みフラッシュ保存する。"""
        box  = self.boxes[box_idx]
        b    = box.box_no
        motor_en_mask    = 0
        init_access_mask = 0

        for ax in AXES:
            i   = AXIS_IDX[ax]
            cfg = box.axes[ax]
            upp = cfg.um_per_pulse
            start_pps = max(1, round(cfg.start_speed_um_s / upp)) if upp != 0.0 else 1

            controller.set_register(ADDR_COEFF_NUM[i],   upp,               box=b)
            controller.set_register(ADDR_COEFF_DEN[i],   1.0,               box=b)
            controller.set_register(ADDR_LIMIT_CW[i],    cfg.limit_cw_um,   box=b)
            controller.set_register(ADDR_LIMIT_CCW[i],   cfg.limit_ccw_um,  box=b)
            controller.set_register(ADDR_HOME_OFFSET[i], cfg.home_offset_um, box=b)
            controller.set_register(ADDR_START[i],       start_pps,         box=b)
            controller.set_register(ADDR_ACCEL[i],       cfg.accel_time_ms, box=b)
            controller.set_register(ADDR_RESOL[i],       cfg.resolution_reg, box=b)
            controller.set_register(ADDR_MOTSEL[i],      cfg.motor_model,   box=b)
            if cfg.motor_en:
                motor_en_mask    |= (1 << i)
            if cfg.init_access:
                init_access_mask |= (1 << i)

        controller.set_register(ADDR_MOTOR_EN,    motor_en_mask,    box=b)
        controller.set_register(ADDR_INIT_ACCESS, init_access_mask, box=b)
        # 役割・台数
        controller.set_register(ADDR_BOX_NO,    box.box_no,               box=b)
        controller.set_register(ADDR_IS_MASTER, 1 if box.is_master else 0, box=b)
        controller.set_register(ADDR_NUM_BOXES, self.num_boxes,           box=b)

        controller.reapply(box=b)          # RP → P_* へ即時反映
        controller.save_registers(box=b)   # RS → フラッシュ保存

    def read_box_from_mcu(self, controller: StageController, box_idx: int) -> None:
        """指定機の全軸パラメータを読み込む。"""
        box = self.boxes[box_idx]
        b   = box.box_no
        try:
            motor_en_mask    = int(float(controller.get_register(ADDR_MOTOR_EN,    box=b)))
            init_access_mask = int(float(controller.get_register(ADDR_INIT_ACCESS, box=b)))
        except (ValueError, TypeError):
            motor_en_mask    = 0
            init_access_mask = 0

        # 役割（IS_MASTER）も MCU から読み戻す
        try:
            box.is_master = int(float(controller.get_register(ADDR_IS_MASTER, box=b))) != 0
        except (ValueError, TypeError):
            pass

        reg_to_microstep = {v: k for k, v in MICROSTEP_TO_REG.items()}

        for ax in AXES:
            i   = AXIS_IDX[ax]
            cfg = box.axes[ax]
            try:
                resol = int(float(controller.get_register(ADDR_RESOL[i], box=b)))
                cfg.microstep      = reg_to_microstep.get(resol, max(1, resol // 10))
                cfg.motor_model    = int(float(controller.get_register(ADDR_MOTSEL[i], box=b)))
                cfg.accel_time_ms  = int(float(controller.get_register(ADDR_ACCEL[i], box=b)))
                upp = float(controller.get_register(ADDR_COEFF_NUM[i], box=b))
                cfg.um_per_fullstep = upp * cfg.microstep
                start_pps          = float(controller.get_register(ADDR_START[i], box=b))
                cfg.start_speed_um_s = start_pps * cfg.um_per_pulse
                cfg.limit_cw_um    = float(controller.get_register(ADDR_LIMIT_CW[i],  box=b))
                cfg.limit_ccw_um   = float(controller.get_register(ADDR_LIMIT_CCW[i], box=b))
                cfg.home_offset_um = float(controller.get_register(ADDR_HOME_OFFSET[i], box=b))
            except (ValueError, TypeError):
                pass
            cfg.motor_en    = bool(motor_en_mask    & (1 << i))
            cfg.init_access = bool(init_access_mask & (1 << i))
