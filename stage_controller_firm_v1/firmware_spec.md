# ファームウェア仕様書

**対象:** stage_controller_firm_v1  
**MCU:** STM32H7A3VIT6（280 MHz）  
**更新日:** 2026-08-09  
**関連:** ピン詳細は [pin_assignment.md](pin_assignment.md)、コマンドは ../command_spec.md 参照

> v0 との差分: ①ドライバ CS/DIR ピン変更 ②EN を N.C. とし励磁は SPI 経由 ③Z軸パルスを
> TIM3→TIM15 に、Y軸を TIM3→TIM8 に変更し**全軸 RCR 搭載タイマ**に統一。

---

## 1. ハードウェア構成

### 1.1 システムクロック
| 項目 | 値 |
|---|---|
| CPU | 280 MHz（HSE Bypass→PLL1） |
| APB1 タイマクロック | 140 MHz |
| APB2 タイマクロック | 280 MHz |

### 1.2 モータードライバ ピン（CVD S-type, SPI2 共有バス）
| 軸 | CS | EN | DIR | PLS(タイマ/ピン) |
|----|----|----|-----|------------------|
| X(0) | PC0 | **N.C.**（SPI励磁） | PA8 | TIM1_CH2 / PA9 |
| Y(1) | PE0 | **N.C.**（SPI励磁） | PC8 | TIM8_CH4 / PC9 |
| Z(2) | PC1 | **N.C.**（SPI励磁） | PE5 | TIM15_CH2 / PE6 |

- **EN ピンは全軸未接続**。励磁は CVD の SPI（NET_IN の ENABLE ビット, SETTING=ENABLE_SRC_SPI）で行う。
- CVD ハンドルは `enable_port=NULL`。`CVD_Enable/DisableMotorPin` は no-op。

### 1.3 ステージセンサ（ON=Low の NC, 内部プルアップ, EXTI 両エッジ）
| 軸 | ORG（原点/未使用）| CCW（−端/原点基準）| CW（＋端）|
|----|------|------|------|
| X | PC11 | PC10 | PA15 |
| Y | PD1  | PD0  | PC12 |
| Z | PD4  | PD3  | PD2  |

- NC センサ: 非検出=Low / 検出・断線=High（`SENSOR_ACTIVE_HIGH=1`）。**未配線ピンは High=検出**と誤読される点に注意。
- 現状 **ORG は未使用**（原点基準は CCW センサ）。**CW ハードリミットは暫定無効**（`LIMIT_CW_ENABLE=0`）。

### 1.4 パルス出力タイマ / DMA
| 軸 | タイマ | CH | Update DMA | PSC | カウンタ | RCR |
|----|--------|----|-----------|-----|---------|-----|
| X | TIM1  | CH2 | DMA1 Stream0 (TIM1_UP)  | 279 | 1 MHz | あり(16bit) |
| Y | TIM8  | CH4 | DMA1 Stream1 (TIM8_UP)  | 279 | 1 MHz | あり(16bit) |
| Z | TIM15 | CH2 | DMA1 Stream2 (TIM15_UP) | 279 | 1 MHz | あり(**8bit**) |

- 全軸 APB2/1 MHz カウンタ。`ARR = 1,000,000 / pps − 1`。
- TIM15 の RCR は 8bit（最大256）。巡航チャンク `CRUISE_CHUNK=256`（RCR=255）は全軸で有効。
- 状態管理: **TIM6**（PSC=1399/ARR=999 → 10 ms）。
- 加減速 DMA 設定: Memory→Peripheral / TIM_DMABASE_ARR / Word / MemInc有 / **Normal** / 優先度 Very High。

### 1.5 SPI2（CVD 設定バス）
Master, 8bit, CPOL=High, CPHA=2Edge, MSB first, Baud ≤1 MHz, NSS=ソフト（手動 CS）。

### 1.6 その他
USB CDC（OTG HS, HSI48）。USART3 = MTD415T TEC ブリッジ（搭載時）。

---

## 2. モーター駆動方式（加減速=DMA / 巡航=連続RCR）

台形プロファイルを3区間に分ける。可変 ARR が要る**加速・減速のみ DMA**（毎パルス, RCR=0）。
一定 ARR の**巡航はタイマを止めず「連続運転」し、RCR で一定パルス毎に UEV 割込**を出して数える。

```
速度[pps]  peak ┌───────────────┐
          加速 /  巡航(一定ARR)  \ 減速
          DMA /   連続運転+RCR    \ DMA
             /                     \
   ─────────┴───── accel_n ─ cruise_n ─ decel_n ─┴─→ パルス
```

| 区間 | ARR | DMA | 終了検知 |
|------|-----|-----|---------|
| ACCEL | 毎パルス可変 | 有(UDE, Normal, RCR=0) | DMAブロックTC |
| CRUISE | 一定(peak) | 無（連続運転） | RCR の UEV 割込 |
| DECEL | 毎パルス可変 | 有(UDE, Normal, RCR=0) | DMAブロックTC |

- **加減速のブロック化**: `MOTOR_DMA_BUF_SIZE(=16386)` を超える区間は複数 Normal DMA ブロックに分割し、各TC（ストリーム停止後＝安全）で次ブロックを起動。
- **巡航のチャンク化**: `RCR = CRUISE_CHUNK-1` を繰り返す。**巡航中はタイマを止めない**ので脱調しない。
  手動ジョグ停止は `stop_req` で現チャンク完了(UEV)を待って `cruise_done` を厳密化し、そこから
  **減速して停止**する（急停止のオーバーシュート低減。位置は厳密）。チャンクサイズは精度非依存で 256（8bit最大）。

> **重要（過去の失敗）**: 巡航を DMA ブロック化する案は、ブロック境界でタイマ停止→ピーク速度で
> 再起動しステッパが脱調（長距離で ~30% 距離不足）したため**撤回**。巡航は連続 RCR 運転が必須。

### 2.1 区間遷移（ハング回避の要）
全遷移は `HAL_TIM_PeriodElapsedCallback → Motor_OnUpdateEvent` に集約し `phase` で分岐:
```
PH_ACCEL : ブロックTC毎 → seg_issued<accel_n:次ブロック / ≥:EnterCruise()
PH_CRUISE: UEV毎        → cruise_remain>0:次チャンクRCR / 0:EnterDecel()
PH_DECEL : ブロックTC毎 → seg_issued<decel_n:次ブロック / ≥:MotorFinish()
```
- 加減速の遷移は「DMA完了TC（ストリーム停止済み）」時点なので `HAL_DMA_Abort` は即時＝安全。
- **EXTI（センサ）からの停止は `MotorAbortISR`** を使い、動作中DMAの Abort（ポーリング）を避ける
  （カウンタ/出力/DMA要求を止めるだけ。ストリーム後始末は次回 `StartRampBlock` の `DMABurst_WriteStop`）。

### 2.2 速度プロファイル（`BuildMotionProfile`）
```
a = (目標速度 − 初速度) / 加速時間[s]           ← 軸別 P_start_pps/P_accel_ms
加速距離 = (peak² − v_start²)/(2a), 減速も対称
accel_n+decel_n>total なら三角形に縮退（peakを距離で制限, 負値は0クランプ）
cruise_n = total − accel_n − decel_n
```
目標速度=`g_speed_pps`（`MOTOR_MAX_PPS=200000` でクランプ）, 初速=`P_start_pps`, 加速時間=`P_accel_ms`。

### 2.3 位置の確定
| 停止契機 | 位置の求め方 | 精度 |
|---------|-------------|------|
| 移動完了 `MotorFinish` | `pos = start_pos ± total_pulses` | **厳密**（指令パルス数） |
| 手動ジョグ停止 `MotorJogStop`（`M<n>F`）| 加減速中=DMA NDTRで即停止／巡航中=`stop_req`→現チャンク完了(UEV)から**減速して停止**（オーバーシュート低減）| **厳密**（離してから最大 256+decel_n パルスぶんブレーキ後に停止, 正確に計上）|
| CCW端 EXTI / ホーミング | `MotorEmittedNow()`＋即停止（安全優先, 過走防止）| 巡航中は ±CRUISE_CHUNK の推定誤差 |

- `MotorEmittedNow` = 現フェーズ実出力 `seg_issued − NDTR`（加減速）＋区間オフセット。
- **CCW ハード端で止まっても pos を 0 にリセットしない**（実出力から実位置を記録）。絶対基準はホーミングで取る。
- D-Cache 無効のため DMA バッファのキャッシュ操作不要。`motor_dma_buf` は 32B アライン。

---

## 3. ホーミング（CCW リミット基準・3段＋オフセット）

原点基準は **CCW（−端）センサ**（ORG は未使用）。各サブ移動は一定速（cruise-only, accel_n=decel_n=0）で
行い EXTI で即停止するため安全。
```
HM_FAST_SEEK : CCWへ高速接近(HOME_FAST_PPS=10000) → CCW作動で停止
HM_BACKOFF   : CWへ低速離脱(HOME_SLOW_PPS=300)    → CCW解除で停止
HM_SLOW_SEEK : CCWへ低速再接近 → CCW作動でセンサ位置を基準に確定
HM_OFFSET    : HOME_OFS>0 なら「センサ位置=-offset」とし CWへ offset パルス移動 → 原点(pos=0)に停止
（HOME_OFS=0 なら HM_SLOW_SEEK でそのまま pos=0）→ 完了で g_homed=1, DONE
異常          : サブ移動がセンサ未検出で自然終了 → HM_ERROR
```
- **原点オフセット**: `HOME_OFS`(parm[56-58], μm) だけ原点(0)を CCW センサから **CW 側**へずらす。
  ホーミング後ステージは原点(0)に停止（リミットスイッチ上に留まらない安全マージン）。
- 進行/判定は EXTI（`HAL_GPIO_EXTI_Callback`）＋ TIM6 10 ms（`MotorStateTick`）で行う。
- `M<n>I`=即時 / `M<n>i`=完了待ち。

---

## 4. リミット

| 種別 | 状態 | 動作 |
|------|------|------|
| ソフト（`P_limit_cw/ccw`）| **ホーミング後のみ**（`g_homed`）| `MotorMoveRel/Jog` で移動量を上限までクランプ |
| CCW ハード（−端センサ）| 常時有効 | 通常移動中に CCW 作動で即停止（`MotorEmittedNow` で位置確定） |
| CW ハード（＋端センサ）| **無効**（`LIMIT_CW_ENABLE=0`）| （センサ不良のため暫定無効。有効化はマクロを1に）|

- 開始前プリチェック: 進行方向の端センサが既に作動中なら移動しない（張り付き時は EXTI エッジが出ないため）。
- 未ホーミング時はソフトリミット非適用（位置不定のため両方向自由）。

---

## 5. コマンド / パラメータ

コマンド一覧は **../command_spec.md**（v0/v1 共通）を参照。主なもの: `MS/mS`（速度）,
`M<n>R/r/A/a`（相対/絶対移動）, `M<n>P`（位置）, `M<n>L`（センサ診断）, `M<n>I/i`（ホーミング）,
`M<n>NP/NM/F`（ジョグ）, `R<addr>R/S`・`RA`・`RS`・`RP`（レジスタ）, `TC*`（TEC）。

### 5.1 パラメータアドレスマップ（`parm[256]`, フラッシュ Bank2 Sector0, CRC32検証）
| アドレス | 名前 | 型 | 単位 | 説明 |
|---|---|---|---|---|
| 0 / 1 | FIRM_VER / BOX_NO | int | — | ファーム番号 / 装置番号 |
| 39 | INIT_TOUT | int | ×10ms | タイムアウト（未使用） |
| 56-58 | HOME_OFS_X/Y/Z | float | μm | ホームオフセット（原点を CCW センサから **CW側へ offset** ずらす。0=センサが原点）|
| 72-74 | LIM_CW_X/Y/Z | float | μm | CW ソフトリミット（既定 25000）|
| 88-90 | LIM_CCW_X/Y/Z | float | μm | CCW ソフトリミット（既定 0）|
| 104-106 | COEF_NUM_X/Y/Z | float | μm | 換算係数 分子 |
| 120-122 | COEF_DEN_X/Y/Z | float | pulse | 換算係数 分母（GUIは常に1.0）|
| 136-138 | START_PPS_X/Y/Z | int | pps | 初速度 |
| 152-154 | ACCEL_MS_X/Y/Z | int | ms | 加速時間 |
| 168-170 | RESOL_X/Y/Z | int | reg | 分解能（RESOLUTION=microstep×10）|
| 183 | INIT_ACC | int | bitmask | 起動時自動ホーミング（未実装連携）|
| 184 | MOTOR_EN | int | bitmask | モーター有効（既定 0b111）|
| 200-202 | MOTSEL_X/Y/Z | int | 16bit | モーター型番（CVD MOT_SEL, 既定 0.75A=0xFE01）|

- `P_motor_coeff = COEF_NUM / COEF_DEN` [μm/pulse]。GUI は「full step長[μm/fullstep]＋分解能」から
  `μm/pulse = full step長 ÷ microstep` を算出し COEF_NUM に書き込む（COEF_DEN=1.0）。GUI は移動量/速度を
  μm/(μm/s) で指定し **PC側で pulse/pps に換算**して `M<n>` を送る。
- **反映**: `R<addr>S`（RAM）→ `RP`（即時反映, 全軸IDLE）→ `RS`（フラッシュ保存）。`Parm_set()` は起動時のみ自動。

---

## 6. CVD ドライバ（SPI 励磁）

初期化 `CVD_ApplyConfigAndEnable`（`Motor_Config` から, `P_motor_en` の軸のみ）:
```
CVD_ApplyConfig: Deactivate → ClearCommError → NET_IN(ENABLE=1) / 電流 / SETTING(ENABLE源=SPI) /
                 RESOLUTION / MOT_SEL を書込 → VerifyConfig(読返し) → Activate → WaitOperationState
CVD_EnableMotorPin: enable_port=NULL のため no-op（励磁は上記 NET_IN で確定）
```
プロファイル `CVD_Profile_PG413M_LA_C`: MOT_SEL=0xFE01(5相0.75A), RESOLUTION=1000(100μstep),
RUN=100% / STOP=50%, SETTING=**ENABLE_SPI**+PLS/DIR, NET_IN=SD_ON|**ENABLE**。

---

## 7. 割り込み優先度
| 割込 | Pri | 用途 |
|------|-----|------|
| DMA1 Stream0/1/2 | 0/0/0※ | X/Y/Z 加減速 DMA 完了（TIM1/8/15 UP） |
| TIM1_UP | 2 | X 巡航 RCR の UEV |
| TIM8_UP_TIM13 | 0 | Y 巡航 RCR の UEV |
| TIM15 | 0 | Z 巡航 RCR の UEV |
| EXTI0-4 / 15_10 | 1 | センサ（原点/端）検出 |
| TIM6_DAC | 3 | 10 ms 状態機械ティック |
| OTG_HS | — | USB |

※実際の NVIC 値は CubeMX(.ioc) 依存。安全停止(EXTI)がパルス割込に確実に割り込めるよう、
必要なら EXTI をパルスタイマと同等以上（数値≤）に設定。

---

## 8. 既知の制約・今後
| 項目 | 状態 |
|------|------|
| 手動ジョグ停止 | 巡航中は減速停止（離してから 256+decel_n パルスぶんブレーキ）。位置は厳密。ドリフト対策の合わせ技 |
| CW ハードリミット | 暫定無効（センサ配線/極性の確認後 `LIMIT_CW_ENABLE=1`）|
| 原点オフセット(HOME_OFS 56-58) | 実装済み（CW側へオフセット, HM_OFFSET）。GUI で μm 設定→ `RP`→ホーミングで有効 |
| ORG センサ | 未使用（原点基準は CCW）|
| `MotorStateTick` | ホーミングFSM進行に使用。アラーム監視等は将来 |
| 起動時自動ホーミング(INIT_ACC) | 未連携 |
