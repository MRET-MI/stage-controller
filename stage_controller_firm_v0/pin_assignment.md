# ピン配置 — stage_controller_firm_v0

**MCU:** STM32H7A3VIT6 ／ **CVDコマンドバス:** SPI2（3軸共有, `hspi2`）
**Enable方式:** EN ピン（GPIO, 負論理）

## モータードライバ制御ピン（CVD S-type）

| 軸 | ドライバ | CS | EN | DIR |
|----|---------|----|----|-----|
| X (0) | CVD1 | PE10 | PE2 | PA8 |
| Y (1) | CVD2 | PE0  | PE1 | PC8 |
| Z (2) | CVD3 | PB0  | PE4 | PC6 |

## パルス出力（PWM タイマー）

| 軸 | タイマー | CH | 出力ピン | DMA (Update) | RCR |
|----|---------|----|---------|--------------|-----|
| X (0) | TIM1  | CH2 | PA9 | DMA1 Stream0 (TIM1_UP) | あり |
| Y (1) | TIM3  | CH4 | PC9 | DMA1 Stream1 (TIM3_UP) | なし（当面未使用） |
| Z (2) | TIM8  | CH2 | PC7 | DMA1 Stream2 (TIM8_UP) | あり |

- 全軸カウンタクロック 1 MHz（TIM1/TIM8=APB2 280MHz/PSC=279、TIM3=APB1 140MHz/PSC=139）。
- 巡航は RCR でパルス計数するため、RCR 非搭載の TIM3(Y) は運転しない。

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
