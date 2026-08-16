# ピン配置 — stage_controller_firm_v1

**MCU:** STM32H7A3VIT6 ／ **CVDコマンドバス:** SPI2（3軸共有, `hspi2`）
**Enable方式:** SPI（NET_IN の ENABLE ビット）。EN ピンは全軸 **N.C.**

## v0 からの変更点
- CVD1_CS: PE10 → **PC0**
- CVD3_CS: PB0 → **PC1**
- CVD3_DIR: PC6 → **PE5**
- Z軸パルス: TIM8_CH2(PC7) → **TIM15_CH2(PE6)**
- Y軸パルス: **TIM3_CH4 → TIM8_CH4**（ピンは PC9 のまま、AF のみ変更）
- CVD1/2/3_EN: GPIO → **N.C.（Enable は SPI 経由）**
- → 全軸 **RCR 搭載タイマ（TIM1/TIM8/TIM15）** に統一し、3軸とも巡航RCRが動作。

## モータードライバ制御ピン（CVD S-type）

| 軸 | ドライバ | CS | EN | DIR |
|----|---------|----|----|-----|
| X (0) | CVD1 | PC0 | N.C.（SPI） | PA8 |
| Y (1) | CVD2 | PE0 | N.C.（SPI） | PC8 |
| Z (2) | CVD3 | PC1 | N.C.（SPI） | PE5 |

## パルス出力（PWM タイマー）

| 軸 | タイマー | CH | 出力ピン | DMA (Update) | RCR |
|----|---------|----|---------|--------------|-----|
| X (0) | TIM1  | CH2 | PA9 | DMA1 Stream0 (TIM1_UP)  | あり |
| Y (1) | TIM8  | CH4 | PC9 | TIM8_UP（ストリームはCubeMXで割当） | あり |
| Z (2) | TIM15 | CH2 | PE6 | TIM15_UP（ストリームはCubeMXで割当） | あり |

- 全軸 APB2（280 MHz）/ PSC=279 → カウンタクロック 1 MHz。
- **全軸 RCR 搭載**のため 3 軸とも巡航（RCR パルス計数）が動作する。
- 巡航の UEV 割込に必要なため、**NVIC で TIM1 / TIM8 / TIM15 のグローバル割込を有効化**すること。
- TIM3 は不使用（v0 で Y に使用していたが TIM8 へ移動）。

## ステージセンサ（ON=Low, 内部プルアップ, EXTI 両エッジ）

| 軸 | ORG（原点） | CCW（−端） | CW（＋端） |
|----|-----------|-----------|-----------|
| X (0) | PC11 | PC10 | PA15 |
| Y (1) | PD1  | PD0  | PC12 |
| Z (2) | PD4  | PD3  | PD2  |

## 状態管理タイマー

| タイマー | 用途 | 周期 |
|---------|------|------|
| TIM6 | 状態機械ティック | 10 ms |
