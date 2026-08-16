# ファームウェア仕様書

**対象:** stage_controller_firm_v0  
**MCU:** STM32H7A3VIT6  
**更新日:** 2026-06-01

---

## 1. ハードウェア構成

### 1.1 システムクロック

| パラメータ | 値 |
|---|---|
| CPU クロック | 280 MHz |
| クロックソース | HSE (Bypass) → PLL1 |
| PLL 設定 | PLLM=5, PLLN=112, PLLP=2 |
| APB1 タイマークロック | 140 MHz |
| APB2 タイマークロック | 280 MHz |

### 1.2 モータードライバ GPIO ピンアサイン

| 信号 | ピン | 説明 |
|------|------|------|
| CVD1_CS | PE10 | 軸0 (X) SPI チップセレクト（負論理） |
| CVD1_EN | PE2 | 軸0 (X) モーター励磁 Enable（負論理） |
| CVD1_DIR | PA8 | 軸0 (X) 回転方向（H=CW） |
| CVD2_CS | PE0 | 軸1 (Y) SPI チップセレクト |
| CVD2_EN | PE1 | 軸1 (Y) Enable |
| CVD2_DIR | PC8 | 軸1 (Y) 回転方向 |
| CVD3_CS | PB0 | 軸2 (Z) SPI チップセレクト |
| CVD3_EN | PE4 | 軸2 (Z) Enable |
| CVD3_DIR | PC6 | 軸2 (Z) 回転方向 |

### 1.3 タイマーと DMA

#### パルス出力タイマー（PWM）

| 軸 | タイマー | チャンネル | PWM ピン | DMA ストリーム | タイマークロック | PSC | カウンタクロック | RCR |
|----|---------|-----------|---------|--------------|----------------|-----|----------------|-----|
| X | TIM1 | CH2 | PA9 | DMA1 Stream0 (TIM1_UP) | APB2 280 MHz | 279 | **1 MHz** | あり |
| Y | TIM8 | CH4 | PC9 | DMA1 Stream2 (TIM8_UP) | APB2 280 MHz | 279 | **1 MHz** | あり |
| Z | TIM3 | CH2 | PC7 | DMA1 Stream1 (TIM3_UP) | APB1 140 MHz | 139 | **1 MHz** | **なし** |

- 全軸カウンタクロックを 1 MHz に統一し、速度 → ARR を `ARR = 1,000,000 / pps − 1` に簡略化。
- **加速・減速**は毎パルス可変 ARR を DMA（RCR=0, UEV毎にDMA）で供給。
- **巡航**は一定 ARR のまま **RCR**（リピティションカウンタ）で一定パルス数を数え、1 回の UEV 割り込みで減速へ移行（DMA不使用）。
- **軸Z(TIM3) は RCR 非搭載**のため巡航カウントが効かず、現HWでは運転しない（将来 TIM15 等へ移行予定）。コードは3軸ともRCR動作前提で記述。

#### 状態管理タイマー

| タイマー | 用途 | PSC | ARR | 周期 |
|---------|------|-----|-----|------|
| TIM6 | 10 ms 状態機械ティック | 1399 | 999 | 10 ms (100 Hz) |

```
周期計算: 140,000,000 Hz / 1400 / 1000 = 100 Hz = 10 ms
```

#### DMA 設定（TIM1/3/8 共通、加減速のみで使用）

| 項目 | 設定値 |
|------|--------|
| 方向 | Memory → Peripheral |
| ペリフェラルアドレス | TIMx_ARR レジスタ（TIM_DMABASE_ARR） |
| バースト長 | 1 ワード（TIM_DMABURSTLENGTH_1TRANSFER） |
| データ幅 | Word (32-bit) |
| メモリインクリメント | 有効 |
| モード | **Normal 固定**（モード切替なし。巡航ではDMA不使用） |
| 優先度 | Very High |

### 1.4 SPI2 設定（CVD Sタイプドライバ向け）

| パラメータ | 値 |
|---|---|
| モード | Master |
| データサイズ | 8 bit |
| CPOL | HIGH (1) |
| CPHA | 2 Edge (1) |
| First Bit | MSB first |
| Baud Rate | ≤ 1 MHz（プリスケーラ 64 分周） |
| NSS | ソフトウェア制御（手動 CS） |

### 1.5 USB

| パラメータ | 値 |
|---|---|
| コントローラ | OTG HS |
| クラス | CDC (Virtual COM Port) |
| クロック | HSI48 |

---

## 2. ソフトウェア構成

### 2.1 ファイル構成（編集対象のみ）

```
Core/
  Inc/
    main.h          ← GPIO ピン名マクロ（CVD1_CS_Pin 等）
    functions.h     ← 型定義・グローバル変数宣言・プロトタイプ
    cvd_driver.h    ← CVD S-type SPI ドライバ ヘッダ
    cvd_profiles.h  ← モータープロファイル定数
  Src/
    main.c          ← 初期化 + メインループ
    functions.c     ← モーター制御（DMA バースト）・コマンドパーサー
    cvd_driver.c    ← CVD SPI ドライバ実装
    cvd_profiles.c  ← モータープロファイル定義
    stm32h7xx_it.c  ← 割り込みハンドラ（DMA / TIM6 / USB）
USB_DEVICE/App/
    usbd_cdc_if.c   ← USB CDC 受信リングバッファ・送信ヘルパー
```

### 2.2 主要グローバル変数

| 変数 | 型 | 説明 |
|------|-----|------|
| `g_cvd[3]` | `CVD_HandleTypeDef *[]` | 3 軸分の CVD ハンドルポインタ配列 |
| `g_motor[3]` | `MotorCtrl_t[]` | 3 軸分のモーター制御状態（フェーズ機械） |
| `g_speed_pps` | `volatile uint32_t` | 速度設定値 [pps]（`MS<n>` で更新） |
| `g_speed_mm` | `volatile float` | 速度設定値 [mm/s]（`mS<n>` で更新） |
| `P_start_pps[3]` | `int32_t[]` | 軸別 初速度 [pps]（parm[136..138]） |
| `P_accel_ms[3]` | `int32_t[]` | 軸別 加速時間 [ms]（parm[152..154]） |
| `motor_dma_buf[3][8192]` | `uint32_t[][]` | 加減速ランプ用 DMA バッファ（32 バイトアライン） |
| `parm[256]` | `reg_t[]` | パラメータレジスタ配列 |
| `g_usb_rx_ready` | `volatile uint8_t` | USB 受信通知フラグ（ISR → main） |

> 加速度は移動毎に `accel = (目標速度 − 初速度) / 加速時間` で算出（グローバル固定値は廃止）。

---

## 3. 処理フロー

### 3.1 全体構成図

```
┌──────────────────────────────────────────────────────────────────────┐
│ 割り込みコンテキスト                                                  │
│                                                                       │
│  USB ISR (OTG_HS_IRQHandler)                                         │
│    └─ CDC_Receive_HS()                                               │
│         ├─ usb_rx_ring[] にバイトを積む                              │
│         └─ '\n' 受信で g_usb_rx_ready = 1                            │
│                                                                       │
│  [加減速] DMA1_Stream0/1/2_IRQHandler → HAL_DMA_IRQHandler           │
│       → TIM_DMAPeriodElapsedCplt → HAL_TIM_PeriodElapsedCallback     │ ← 加減速ブロック完了
│  [巡航]   TIM1_UP / TIM8_UP_TIM13 / TIM3_IRQHandler                  │
│       → HAL_TIM_IRQHandler → HAL_TIM_PeriodElapsedCallback           │ ← 巡航RCRのUEV
│            └→ Motor_OnUpdateEvent()  ← phaseで分岐し区間遷移          │
│                                                                       │
│  TIM6_DAC_IRQHandler (10 ms)                                         │
│    └─ MotorStateTick()  ← 将来: アラーム監視・タイムアウト           │
└──────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────┐
│ メインループコンテキスト                                              │
│                                                                       │
│  while(1)                                                            │
│    └─ Cmd_Process()                                                  │
│         ├─ CDC_RxReadByte() でリングバッファを順次読み出し            │
│         ├─ '\n' でラインを区切り Cmd_Execute() を呼ぶ                │
│         └─ CDC_SendString() で応答を送信                             │
└──────────────────────────────────────────────────────────────────────┘
```

### 3.2 起動シーケンス（main.c USER CODE BEGIN 2）

```
HAL 初期化・クロック設定・ペリフェラル初期化（CubeMX 自動生成）
  ↓
Motor_Init()
  ├─ g_motor[] を IDLE にリセット
  └─ P_limit_cw/ccw を INT32_MAX/MIN（実質無制限）に設定
  ↓
GetCLK()
  └─ APB1/APB2 タイマークロックを計測して変数に保存
  ↓
Reg_prepare() → Parm_Flash_Load() → Parm_set()
   └─ parm[] 既定値 → フラッシュ値で上書き → 適用＋Motor_Config()
  ↓
TIM6 PSC=1399 / ARR=999 を設定 → 10 ms 周期確定
  ↓
DMA 割り込み優先度（DMA1_Stream0/1/2: Pri=3）
TIM_UP 割り込み有効化（TIM1_UP / TIM8_UP_TIM13 / TIM3: Pri=3, 巡航UEV用）
TIM6 NVIC（Pri=4）・HAL_TIM_Base_Start_IT(&htim6)
  ↓
while(1) → Cmd_Process()   ※モーター駆動は完全に割り込み駆動（ポーリング不要）
```

### 3.3 コマンド受信・応答フロー

```
[GUI → USB → STM32]

1. USB ISR が起動
   CDC_Receive_HS(Buf, Len) が呼ばれる
   → Buf の内容を usb_rx_ring[] に追記
   → '\n' が含まれていれば g_usb_rx_ready = 1

2. メインループ: Cmd_Process()
   CDC_RxReadByte() でバイトを 1 文字ずつ取り出す
   → '\r' は読み飛ばし
   → '\n' でラインが完成 → Cmd_Execute(cmd_line) を呼ぶ
   → 文字数が 127 を超えたら切り捨て

3. Cmd_Execute(line)
   ';' で分割して各サブコマンドを ParseSubCmd() に渡す
   → 最後に出力を持つサブコマンドの結果を resp[] に格納
   → resp が空なら "OK\n"、それ以外は "resp\n" を送信

4. CDC_SendString(str)
   前回送信完了を待ってから CDC_Transmit_HS() を呼ぶ

[STM32 → USB → GUI]
```

### 3.4 モーター制御フロー（加減速DMA / 巡航RCR の3区間方式）

```
ParseSubCmd("M0R500")
  ↓
MotorMoveRel(axis=0, pulses=500, wait=0)
  ↓
MotorPrepare()
  ├─ pos を保持しつつ状態初期化、speed を MOTOR_MAX_PPS でクランプ
  ├─ start_pps / accel_pps2(=（目標−初速)/加速時間) を設定
  └─ BuildMotionProfile() → accel_n / cruise_n / decel_n（整数）を確定

StartMotionDma(axis=0)
  ├─ CVD_SetDirCW/CCW で DIR 設定、start_pos = pos
  └─ phase = PH_ACCEL → StartRampSegment()（加速ブロック開始）

wait=0 なので即時 "OK\n" を返す（wait付き小文字は state==IDLE まで待機）

[区間遷移は全て HAL_TIM_PeriodElapsedCallback → Motor_OnUpdateEvent で進行]

PH_ACCEL: 毎パルスDMA(RCR=0)で可変ARR供給。ブロックTC毎に
   seg_issued < accel_n → 次ブロック / seg_issued≥accel_n → EnterCruise()

PH_CRUISE: EnterCruise() が ARR=peak固定・RCR=cruise_n-1・UEV割込有効化（DMA停止）
   タイマは止めずシームレス。RCR+1パルス後のUEVで
   cruise_remain>0 → 次チャンクRCR / ==0 → EnterDecel()
   （>65536 は MOTOR_RCR_MAX 単位でチャンク分割）

PH_DECEL: 毎パルスDMA(RCR=0)で可変ARR供給。最終ブロックTCで
   MotorFinish() → MotorHwStop（DMA/PWM/タイマ停止, RCR=0）
                 → pos = start_pos ± total_pulses（区間カウントで正確）
                 → state = MOTOR_IDLE
```

**ハング回避**: 全ての区間遷移は「DMA完了TC（ストリーム停止済み）」または
「巡航UEV（DMA非動作）」の時点で実行されるため、動作中DMAの
`HAL_DMA_Abort/DeInit` が発生せず構造的にハングしない。

---

## 4. モーター状態機械

### 4.1 状態遷移

```
              MotorMoveRel/Abs()
  ┌────────────────────────────────────────────┐
  │                                            ↓
IDLE ────────────────────────────────────→ MOVING ──→ DMA 完了 ──→ IDLE
  │   MotorHome()                              │
  ├────────────────────────────────────────→ HOMING ──→ DMA 完了 ──→ IDLE (pos=home_offset)
  │   MotorJog()                               │         ソフトリミット到達
  └────────────────────────────────────────→ JOGGING ──→ MotorJogStop() ──→ IDLE
```

MOVING / HOMING / JOGGING は ACCEL→CRUISE→DECEL の3区間で駆動される。
`MotorJogStop()` は出力済みパルス数を概算して位置をコミットし停止する。
完了時の位置は `pos = start_pos ± total_pulses`（区間カウントで正確に確定）。

### 4.2 MotorCtrl_t 構造体

```c
typedef enum { PH_IDLE, PH_ACCEL, PH_CRUISE, PH_DECEL } MotorPhase_t;

typedef struct {
    /* 軸状態 */
    volatile MotorState_t   state;
    volatile int32_t        pos;           // 現在位置 [パルス]（移動間で保持）
    int32_t                 start_pos;     // 移動開始時の pos
    int32_t                 target;        // 予約
    uint8_t                 dir;           // 1=CW, 0=CCW

    /* 速度プロファイル */
    uint32_t                total_pulses;  // 今回の移動総パルス数
    float                   peak_speed;    // 到達ピーク速度 [pps]
    float                   start_pps;     // 初速/終速 [pps]
    float                   accel_pps2;    // 加速度 [pps²]
    float                   accel_pulses;  // 加速距離(float, GetSpeedAtPulse用)
    float                   decel_pulses;  // 減速距離(float)

    /* 整数区間パルス数 */
    uint32_t                accel_n;       // 加速パルス数
    uint32_t                cruise_n;      // 巡航パルス数
    uint32_t                decel_n;       // 減速パルス数

    /* 実行時フェーズ状態 */
    volatile MotorPhase_t   phase;         // PH_IDLE/ACCEL/CRUISE/DECEL
    uint32_t                seg_issued;    // 現ランプ区間で発行済みパルス数
    volatile uint32_t       cruise_remain; // RCRで数え残す巡航パルス数
    uint32_t                block_pulses;  // 進行中DMAブロックのパルス数
    volatile uint8_t        dma_running;
} MotorCtrl_t;
```

### 4.3 ソフトウェアリミット

- `P_limit_cw[axis]`  — CW 方向の上限位置 [パルス]
- `P_limit_ccw[axis]` — CCW 方向の下限位置 [パルス]
- デフォルト（起動直後）: `INT32_MAX` / `INT32_MIN`（実質無制限）
- `Parm_set()` 呼び出し後: `parm[72..74]` / `parm[88..90]` から `P_motor_coeff` で換算

---

## 5. モーター駆動方式（加減速=DMA / 巡航=RCR）

### 5.1 基本原理

台形プロファイルを3区間に分け、可変ARRが必要な**加速・減速のみ DMA**、一定ARRの
**巡航は RCR（リピティションカウンタ）でパルス数を数える**。これにより巡航中はCPU・DMA
ともに非介入で、移動完了通知も含め完全に割り込み駆動（while(1)ポーリング不要）。

```
速度 [pps]
    ↑  peak_speed   ┌──────────────────┐
    |              /  巡航(一定ARR/RCR)  \
    |   加速(DMA) /                       \ 減速(DMA)
    |  RCR=0     /                         \ RCR=0
    |          /                            \
    +─────────┴──────────────────────────────┴── パルス
        accel_n        cruise_n         decel_n
```

- パルスは PWM 1周期 = 1ステップ。CCR は `SpeedToArr(peak)/2`（最小ARRの半分, 常に CCR≤ARR）。
- `ARR = 1,000,000 / speed[pps] − 1`（全軸1MHzカウンタで共通）。

### 5.2 区間別の動作

| 区間 | ARR | UEV(更新イベント) | DMA | 終了検知 |
|------|-----|------------------|-----|---------|
| ACCEL | 毎パルス可変 | RCR=0（毎パルス） | 有（UDE, ノーマル） | DMAブロックTC |
| CRUISE | 一定(peak) | RCR+1毎に1回 | 無 | RCRのUEV割込 |
| DECEL | 毎パルス可変 | RCR=0（毎パルス） | 有（UDE, ノーマル） | DMAブロックTC |

- **加減速のブロック化**: 加速/減速テーブルが `MOTOR_DMA_BUF_SIZE(=8192)` を超える場合は
  複数ノーマルDMAブロックに分割し、各ブロックTC（ストリーム停止後＝安全）で次を起動。
- **巡航のチャンク化**: `cruise_n > MOTOR_RCR_MAX(=65536)` の場合は RCR を最大値で繰り返し、
  UEV割込（最大65536パルス毎≒328ms@200kHz）でチャンクを数える。CPU負荷は極小。

### 5.3 速度プロファイル（`BuildMotionProfile`）

```
加速度 accel = (目標速度 − 初速度) / 加速時間[s]   ← 軸別 P_accel_ms から
加速距離 = (peak² − v_start²) / (2·accel)
減速距離 = (peak² − v_end²)   / (2·accel)      （v_end = v_start, 対称）
accel_n + decel_n > total なら三角形に縮退（peakを距離で制限）
cruise_n = total − accel_n − decel_n
各パルス瞬間速度 v(s): 加速 √(v_start²+2·accel·s) / 定速 peak / 減速 √(v_end²+2·accel·(total−s))
```

| 入力 | 由来 |
|------|------|
| 目標速度 [pps] | `g_speed_pps`（GUIが μm/s→pps 換算して `MS<n>`）。`MOTOR_MAX_PPS` でクランプ |
| 初速度 [pps] | `P_start_pps[axis]`（parm[136..138]） |
| 加速時間 [ms] | `P_accel_ms[axis]`（parm[152..154]） |

### 5.4 区間遷移（ハング回避の要）

すべての遷移は **HAL_TIM_PeriodElapsedCallback → Motor_OnUpdateEvent** に集約し、`phase` で分岐:

```
PH_ACCEL: ブロックTC毎 → seg_issued<accel_n: 次ブロック / ≥accel_n: EnterCruise()
PH_CRUISE: UEV毎 → cruise_remain>0: 次チャンクRCR / ==0: EnterDecel()
PH_DECEL: ブロックTC毎 → seg_issued<decel_n: 次ブロック / ≥decel_n: MotorFinish()
```

- ACCEL→CRUISE: 加速最終TC（DMA停止済み）で UDE停止→ARR=peak→RCR設定→UEV割込有効。
  タイマは止めずシームレス。
- CRUISE→DECEL: 巡航UEV（DMA非動作）で UEV割込停止→RCR=0→減速DMA開始。
- いずれも**動作中DMAを Abort/DeInit しない**ため、旧循環方式のハングが構造的に発生しない。

### 5.5 割り込みルーティング

| 区間 | 割り込み源 | 経路 |
|------|-----------|------|
| ACCEL/DECEL | DMA1_Stream0/1/2（TC） | HAL_DMA_IRQHandler → TIM_DMAPeriodElapsedCplt → HAL_TIM_PeriodElapsedCallback |
| CRUISE | TIM1_UP / TIM8_UP_TIM13 / TIM3 | HAL_TIM_IRQHandler → HAL_TIM_PeriodElapsedCallback |

`HAL_TIM_PeriodElapsedCallback`（functions.c でオーバーライド）が `Motor_OnUpdateEvent` を呼ぶ。
`HAL_TIM_PeriodElapsedHalfCpltCallback` は未使用（空）。

### 5.6 D-キャッシュ

本プロジェクトは D-Cache 無効のため、DMAバッファのキャッシュメンテナンス
（`SCB_CleanDCache_by_Addr`）は不要・呼ばない（CPU書き込みは直接RAMへ反映）。
`motor_dma_buf` は `__attribute__((aligned(32)))` 配置。

---

## 6. CVD ドライバ

### 6.1 初期化シーケンス（CVD_ApplyConfigAndEnable）

```
CVD_DisableMotorPin()    ← EN を Hi（励磁オフ）
CVD_Deactivate()         ← DEACTIVATE コマンド送信
CVD_ClearCommError()     ← 通信エラークリア
CVD_WriteReg16(NET_IN)   ← SD/FIL ビット設定
CVD_WriteReg16(RUN_CRNT / STOP_CRNT / SETTING / RESOLUTION / MOT_SEL)
CVD_VerifyConfig()       ← 書き込み値の読み返しチェック
CVD_Activate()           ← ACTIVATE コマンド送信（30 ms 待ち）
CVD_WaitOperationState() ← STATUS1.bit15 が立つまで待つ（最大 100 ms）
CVD_EnableMotorPin()     ← EN を Lo（励磁オン）
```

### 6.2 使用プロファイル

`CVD_Profile_PG413M_LA_C`（= `CVD_Profile_CVD5_0P75A_100Microstep`）

| パラメータ | 値 | 意味 |
|---|---|---|
| MOT_SEL | 0xFE01 | 5相 0.75A モーター |
| RESOLUTION | 1000 (=50000/50) | 100 マイクロステップ |
| RUN_CURRENT | 1000 (=100.0%) | 運転電流 100% |
| STOP_CURRENT | 500 (=50.0%) | 停止電流 50% |
| SETTING | PLS/DIR + SPI制御 | |
| NET_IN | SD ON, Filter OFF | |

---

## 7. パラメータレジスタ (`parm[]`)

`parm[256]` 配列はフラッシュ Bank 2, Sector 0（0x08100000, 8 KB）に保存される。

### 7.1 フラッシュレイアウト

```
0x08100000  [16 B]  ヘッダー (magic=0x50524D41 / size / CRC32 / reserved)
0x08100010  [7168 B] parm[256] の生データ
合計 7184 B < 8192 B (セクターサイズ) ✓
```

- フラッシュワード: 128 bit = **16 bytes** で書き込み
- `HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, data_addr)`
- 書き込み前に CRC32 を計算してヘッダーに記録、読み込み時に検証

### 7.2 パラメータアドレスマップ

**単位系: 距離 [μm], 速度 [μm/s]**

| アドレス | 名前 | フィールド | 型 | 単位 | 説明 |
|---------|------|-----------|-----|------|------|
| 0 | firm_no | data | int | — | ファームウェア番号 |
| 1 | box_no | data | int | — | 装置番号 |
| 39 | init_timeout | data | int | 10 ms | ホーミングタイムアウト（将来用・未使用） |
| **56** | home_offset_x | data_f | float | **μm** | 軸X ホームオフセット |
| **57** | home_offset_y | data_f | float | **μm** | 軸Y ホームオフセット |
| **58** | home_offset_z | data_f | float | **μm** | 軸Z ホームオフセット |
| **72** | limit_cw_x | data_f | float | **μm** | 軸X CW ソフトリミット |
| **73** | limit_cw_y | data_f | float | **μm** | 軸Y CW ソフトリミット |
| **74** | limit_cw_z | data_f | float | **μm** | 軸Z CW ソフトリミット |
| **88** | limit_ccw_x | data_f | float | **μm** | 軸X CCW ソフトリミット |
| **89** | limit_ccw_y | data_f | float | **μm** | 軸Y CCW ソフトリミット |
| **90** | limit_ccw_z | data_f | float | **μm** | 軸Z CCW ソフトリミット |
| **104** | coeff_num_x | data_f | float | **μm** | 軸X 係数分子 |
| **105** | coeff_num_y | data_f | float | **μm** | 軸Y 係数分子 |
| **106** | coeff_num_z | data_f | float | **μm** | 軸Z 係数分子 |
| **120** | coeff_den_x | data_f | float | **pulse** | 軸X 係数分母 |
| **121** | coeff_den_y | data_f | float | **pulse** | 軸Y 係数分母 |
| **122** | coeff_den_z | data_f | float | **pulse** | 軸Z 係数分母（GUIは常に1.0） |
| **136-138** | start_pps_x/y/z | data | int | **pps** | 各軸 初速度 |
| **152-154** | accel_ms_x/y/z | data | int | **ms** | 各軸 加速時間 |
| **168-170** | resol_x/y/z | data | int | reg | 各軸 分解能（RESOLUTION = microstep×10） |
| **200-202** | motsel_x/y/z | data | int | 16bit | 各軸 モーター型番（MOT_SEL値） |
| 183 | init_access | data | int | bitmask | ホーミング実施 (bit0=軸0…) |
| 184 | motor_en | data | int | bitmask | モーター有効 (bit0=軸0…) |

> **分解能と um/pulse の関係（重要）**: GUI は「ねじリード [μm/回転]」と「分解能(microstep)」を真の入力値とし、
> `um/pulse = リード ÷ (microstep × 500)` を自動計算して COEF_NUM(104-106) に書き込む（COEF_DEN は常に 1.0）。
> 分解能 RESOLUTION(168-170) と um/pulse は物理的に連動するため、GUI が両者を同一ソースから算出することで整合を保証する。
> パルス周波数は MCU で `ARR = 1MHz / pps − 1`（pps = 速度[μm/s] ÷ um/pulse, GUI 換算）として生成され、分解能は um/pulse 経由で間接的に反映される。

### 7.x モーション・モーター設定（補足）

- **初速度 / 加速時間**: 台形プロファイルの加速度は `a = (目標速度 − 初速度) / 加速時間[s]` で move ごとに算出。
  減速側も初速度まで対称に減速する。
- **分解能 (RESOLUTION)**: microstep(1/2/4/5/10/20/50/100) → レジスタ値 ×10。起動時に `Motor_Config()` が CVD へ SPI 設定。
- **モーター型番 (MOT_SEL)**: 0.35A=0xFF00 / 0.75A=0xFE01 / 1.20A=0xFD02 / 1.40A=0xFC03 / 1.80A=0xFB04 / 2.40A=0xFA05。
- これら（分解能・型番）の変更はフラッシュ保存後の**再起動時**に反映される（実行中の再適用はしない）。

### 7.3 換算係数の計算

```
P_motor_coeff[n] = parm[104+n].data_f / parm[120+n].data_f   [μm/pulse]

物理量コマンド m<n>R<dist_μm> の内部処理:
  pulses = dist_μm  / P_motor_coeff[n]
  pps    = speed_μm_s / P_motor_coeff[n]
```

**設定例（1 μm/pulse のステージ）:**
```
parm[104].data_f = 1.0   (coeff_num = 1 μm)
parm[120].data_f = 1.0   (coeff_den = 1 pulse)
→ P_motor_coeff[0] = 1.0 μm/pulse
```

**設定例（0.1 μm/pulse のステージ）:**
```
parm[104].data_f = 0.1   または  1.0
parm[120].data_f = 1.0          10.0
→ P_motor_coeff[0] = 0.1 μm/pulse
```

### 7.4 GUI からの設定手順

1. GUI「設定…」ボタン → ステージ設定ダイアログを開く
2. 各軸の係数・リミット・ホームオフセットを入力
3. 「MCUに書き込み・保存」ボタン
   - `R<addr>S<val>` コマンドで各レジスタに書き込み
   - `RS` コマンドでフラッシュへ保存
4. 次回起動時に MCU が自動でフラッシュから読み込む（`Parm_Flash_Load()` → `Parm_set()`）

---

## 8. 割り込み優先度

| 割り込み | IRQn | PreemptPriority | 用途 |
|---------|------|--------|------|
| OTG_HS | OTG_HS_IRQn | CubeMX 設定値 | USB データ受信・送信 |
| DMA1 Stream0 | DMA1_Stream0_IRQn | 3 | TIM1 UP DMA（軸X 加減速） |
| DMA1 Stream1 | DMA1_Stream1_IRQn | 3 | TIM3 UP DMA（軸Z 加減速） |
| DMA1 Stream2 | DMA1_Stream2_IRQn | 3 | TIM8 UP DMA（軸Y 加減速） |
| TIM1 Update | TIM1_UP_IRQn | 3 | 軸X 巡航 RCR の UEV |
| TIM8 Update/TIM13 | TIM8_UP_TIM13_IRQn | 3 | 軸Y 巡航 RCR の UEV |
| TIM3 | TIM3_IRQn | 3 | 軸Z 巡航（現HWは運転しない） |
| TIM6 / DAC | TIM6_DAC_IRQn | 4 | 10 ms 状態機械ティック |

> 加減速は DMA 完了割り込み、巡航は TIM_UP（RCRゲート）割り込みで進行。両者とも
> `HAL_TIM_PeriodElapsedCallback → Motor_OnUpdateEvent` に集約され phase で分岐する。

---

## 9. 未実装・今後の課題

| 項目 | 状態 | 備考 |
|------|------|------|
| フラッシュ読み書き（`RS` コマンド）| 実装済 | `Parm_Flash_Save/Load`（Bank2 Sector0, CRC32検証） |
| 物理的ホームセンサー検出 | 未実装 | ソフトリミット到達で代替中 |
| `MotorStateTick()` (TIM6) の活用 | 空実装 | タイムアウト・CVD アラーム検出等を追加予定 |
| 動作中コマンドへの `NG` 応答 | 無視（暗黙） | 必要なら `ParseSubCmd` で明示的に `NG` を返すよう修正 |
| **軸Z(TIM3) の巡航** | 非対応 | TIM3 は RCR 非搭載。現HWでは軸Zを運転しない（将来 TIM15 化で解消） |
| 加減速/巡航境界のパルス整合 | 要実機検証 | 毎パルスDMAのプリロード/末尾やブロック境界の波形をオシロで確認・微調整 |
| ジョグ停止位置 | 概算 | 手動停止のため `MotorJogStop` は出力済みパルスを概算してコミット |
