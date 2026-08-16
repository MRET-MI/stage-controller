/*
 * functions.c — Motor control (DMA burst) + command parser
 *
 *  Created on: 2026/05/26
 *      Author: tm472
 */

#include "functions.h"
#include "cvd_profiles.h"
#include "mtd415t.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;    /* v1: Y軸パルス TIM8 CH4(PC9) */
extern TIM_HandleTypeDef htim15;   /* v1: Z軸パルス TIM15 CH2(PE6) */
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi1;    /* カスケード接続バス (PB3/PB4/PB5) */

/* ── Clock variables ──────────────────────────────────────────────────────── */

uint32_t sysclk_hz;
uint32_t apb1_clocks,       apb2_clocks;
uint32_t apb1_timer_clocks, apb2_timer_clocks;
uint32_t tim1_psc, tim6_psc, tim8_psc, tim15_psc;
uint32_t tim1_clk, tim6_clk, tim8_clk, tim15_clk;

/* ── Motor CVD handles ────────────────────────────────────────────────────────
 * v1: EN ピンは全軸 N.C.。Enable はドライバの SPI(NET_IN ENABLE ビット)で行うため
 *     enable_port=NULL とし、CVD_Enable/DisableMotorPin は no-op になる。*/

CVD_HandleTypeDef g_axis1_cvd = {
    .hspi = &hspi2,
    .cs_port = CVD1_CS_GPIO_Port, .cs_pin = CVD1_CS_Pin,
    .enable_port = NULL, .enable_pin = 0u,          /* EN=N.C. → SPI Enable */
    .dir_port = CVD1_DIR_GPIO_Port, .dir_pin = CVD1_DIR_Pin,
};
CVD_HandleTypeDef g_axis2_cvd = {
    .hspi = &hspi2,
    .cs_port = CVD2_CS_GPIO_Port, .cs_pin = CVD2_CS_Pin,
    .enable_port = NULL, .enable_pin = 0u,          /* EN=N.C. → SPI Enable */
    .dir_port = CVD2_DIR_GPIO_Port, .dir_pin = CVD2_DIR_Pin,
};
CVD_HandleTypeDef g_axis3_cvd = {
    .hspi = &hspi2,
    .cs_port = CVD3_CS_GPIO_Port, .cs_pin = CVD3_CS_Pin,
    .enable_port = NULL, .enable_pin = 0u,          /* EN=N.C. → SPI Enable */
    .dir_port = CVD3_DIR_GPIO_Port, .dir_pin = CVD3_DIR_Pin,
};
CVD_HandleTypeDef *g_cvd[MOTOR_MAX] = { &g_axis1_cvd, &g_axis2_cvd, &g_axis3_cvd };

/* ── Motor state & DMA buffers ────────────────────────────────────────────── */

MotorCtrl_t g_motor[MOTOR_MAX];

/*
 * DMA buffers: one ARR word per pulse, aligned to D-cache line (32 bytes).
 * SCB_CleanDCache_by_Addr is called before each DMA transfer.
 */
static uint32_t motor_dma_buf[MOTOR_MAX][MOTOR_DMA_BUF_SIZE]
    __attribute__((aligned(32)));

volatile uint32_t g_speed_pps  = 1000u;
volatile float    g_speed_mm   = 1.0f;

/* 軸別 CVD 設定（Parm_set で parm[] から展開、Motor_Config で適用） */
static CVD_ConfigTypeDef g_cvd_cfg[MOTOR_MAX];

/* ── Timer / axis mapping ─────────────────────────────────────────────────── */
/*
 * axis 0 (X): TIM1  CH2 (PA9),  APB2 (280 MHz), PSC=279 → 1 MHz counter  [RCR 有]
 * axis 1 (Y): TIM8  CH4 (PC9),  APB2 (280 MHz), PSC=279 → 1 MHz counter  [RCR 有]
 * axis 2 (Z): TIM15 CH2 (PE6),  APB2 (280 MHz), PSC=279 → 1 MHz counter  [RCR 有]
 *
 * v1: 全軸を RCR 搭載タイマ（TIM1/TIM8/TIM15）に統一。3軸とも巡航RCRが動作する。
 * （Y は PC9 のまま AF を TIM3→TIM8_CH4 に変更）
 */
static TIM_HandleTypeDef * const k_tim[MOTOR_MAX] = { &htim1, &htim8, &htim15 };
static const uint32_t k_ch[MOTOR_MAX] = { TIM_CHANNEL_2, TIM_CHANNEL_4, TIM_CHANNEL_2 };

/* 巡航は一定ARRで「連続運転」(RCR計数, タイマを止めない)。DMA化するとブロック境界で
 * タイマ停止→再起動が入り、ピーク速度からの再起動で脱調するため DMA化しない。
 * 手動ジョグ停止は「現チャンク完了時(UEV)に停止」する方式(stop_req)で計数誤差ゼロにするため、
 * チャンクサイズは精度に依存しない → 8bit RCR 最大の 256(RCR=255, 全軸で有効)にして
 * 巡航中のUEV割込回数を最小化する。*/
#define CRUISE_CHUNK   256u

static uint8_t FindAxisByTim(const TIM_HandleTypeDef *htim, uint8_t *axis_out)
{
    uint8_t i;
    for (i = 0u; i < MOTOR_MAX; i++) {
        if (k_tim[i]->Instance == htim->Instance) {
            *axis_out = i;
            return 1u;
        }
    }
    return 0u;
}

/* ── センサ（ORG/CW/CCW, ON=Low, 内部プルアップ, EXTI両エッジ） ─────────────── */

static GPIO_TypeDef * const k_org_port[MOTOR_MAX] = { ORG1_GPIO_Port, ORG2_GPIO_Port, ORG3_GPIO_Port };
static const uint16_t       k_org_pin [MOTOR_MAX] = { ORG1_Pin,       ORG2_Pin,       ORG3_Pin };
static GPIO_TypeDef * const k_cw_port [MOTOR_MAX] = { CW1_GPIO_Port,  CW2_GPIO_Port,  CW3_GPIO_Port };
static const uint16_t       k_cw_pin  [MOTOR_MAX] = { CW1_Pin,        CW2_Pin,        CW3_Pin };
static GPIO_TypeDef * const k_ccw_port[MOTOR_MAX] = { CCW1_GPIO_Port, CCW2_GPIO_Port, CCW3_GPIO_Port };
static const uint16_t       k_ccw_pin [MOTOR_MAX] = { CCW1_Pin,       CCW2_Pin,       CCW3_Pin };

/* ── 汎用 GPIO 出力（PE9..PE12, GUI トグルで制御。極性は PC 側で適用, ここは生レベル） ── */
#define GPO_MAX 4u
static GPIO_TypeDef * const k_gpo_port[GPO_MAX] = { GPO0_GPIO_Port, GPO1_GPIO_Port, GPO2_GPIO_Port, GPO3_GPIO_Port };
static const uint16_t       k_gpo_pin [GPO_MAX] = { GPO0_Pin,       GPO1_Pin,       GPO2_Pin,       GPO3_Pin };

/* ホーミング状態（軸別） */
volatile HomingState_t g_homing[MOTOR_MAX]  = { HM_IDLE, HM_IDLE, HM_IDLE };
static volatile HomeEvt_t g_home_evt[MOTOR_MAX] = { HEVT_NONE, HEVT_NONE, HEVT_NONE };

/* ホーミング済みフラグ（軸別）。ホーミング前は位置(pos)が不定なので
 * ソフトリミットを適用せず両方向に自由移動させる（Motor_Init で 0、HM_DONE で 1）。*/
static volatile uint8_t g_homed[MOTOR_MAX] = { 0u, 0u, 0u };

/* 検出レベルは SENSOR_ACTIVE_HIGH で切替（NC=駿河精機は検出=High） */
static inline uint8_t SensorActive(GPIO_TypeDef *port, uint16_t pin)
{
#if SENSOR_ACTIVE_HIGH
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)   ? 1u : 0u;
#else
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1u : 0u;
#endif
}
static inline uint8_t OrgActive(uint8_t a) { return SensorActive(k_org_port[a], k_org_pin[a]); }
static inline uint8_t CwActive (uint8_t a) { return SensorActive(k_cw_port[a],  k_cw_pin[a]);  }
static inline uint8_t CcwActive(uint8_t a) { return SensorActive(k_ccw_port[a], k_ccw_pin[a]); }

/* センサGPIO/EXTI(両エッジ)/NVIC は CubeMX(MX_GPIO_Init)で設定済み。
 * ここではテーブルと読み取りヘルパのみ提供する。 */

/* ── Motion profile helpers ───────────────────────────────────────────────── */

static float ClampSpeed(float s)
{
    return (s < MOTOR_SPEED_MIN_F) ? MOTOR_SPEED_MIN_F : s;
}

static float SafeSqrt(float v)
{
    return (v <= 0.0f) ? MOTOR_SPEED_MIN_F : sqrtf(v);
}

/*
 * Convert speed [pps] to ARR value for 1 MHz counter.
 * ARR = 1,000,000 / pps - 1  (minimum 1)
 */
static uint32_t SpeedToArr(float speed_pps)
{
    uint32_t arr;
    speed_pps = ClampSpeed(speed_pps);
    arr = (uint32_t)(((float)MOTOR_TIM_CLK_HZ / speed_pps) + 0.5f);
    if (arr < 2u) arr = 2u;
    return arr - 1u;
}

/*
 * Build trapezoidal velocity profile and整数の区間パルス数を確定する。
 * 初速度・加速度は呼び出し前に MotorPrepare で m->start_pps / m->accel_pps2 に設定済み。
 * 出力: peak_speed, accel_pulses/decel_pulses(float, GetSpeedAtPulse用),
 *       accel_n/cruise_n/decel_n(整数, 区間制御用)。
 */
static void BuildMotionProfile(uint8_t axis, uint32_t total, float speed)
{
    MotorCtrl_t *m = &g_motor[axis];
    float v_start = ClampSpeed(m->start_pps);
    float v_end   = v_start;
    float v_tgt   = ClampSpeed(speed);
    float a       = ClampSpeed(m->accel_pps2);

    float d_acc = (v_tgt * v_tgt - v_start * v_start) / (2.0f * a);
    float d_dec = (v_tgt * v_tgt - v_end   * v_end)   / (2.0f * a);
    float total_f = (float)total;

    if ((d_acc + d_dec) <= total_f) {
        m->peak_speed   = v_tgt;
        m->accel_pulses = d_acc;
        m->decel_pulses = d_dec;
    } else {
        /* Triangular profile: peak speed limited by distance */
        float peak_sq = (2.0f * total_f * a + v_start * v_start + v_end * v_end) / 2.0f;
        float vp = SafeSqrt(peak_sq);
        if (vp > v_tgt) vp = v_tgt;
        m->peak_speed   = vp;
        m->accel_pulses = (vp * vp - v_start * v_start) / (2.0f * a);
        m->decel_pulses = (vp * vp - v_end   * v_end)   / (2.0f * a);
    }

    /* 初速度 > 目標速度 等で accel/decel が負になり得る。
     * 負の float を (uint32_t) 変換すると巨大値になり cruise_n がアンダーフロー＝暴走するため
     * 0 にクランプしておく（この場合は実質その区間なし＝一定速で total を消化）。*/
    if (m->accel_pulses < 0.0f) m->accel_pulses = 0.0f;
    if (m->decel_pulses < 0.0f) m->decel_pulses = 0.0f;

    /* 整数区間数を確定（accel_n + decel_n ≤ total, 残りを cruise_n） */
    uint32_t an = (uint32_t)(m->accel_pulses + 0.5f);
    uint32_t dn = (uint32_t)(m->decel_pulses + 0.5f);
    if (an + dn > total) {
        /* 丸め誤差/三角形で超過したら按分 */
        if (an + dn == 0u) { an = total; dn = 0u; }
        else {
            uint32_t scaled_a = (uint32_t)(((uint64_t)an * total) / (an + dn));
            an = scaled_a;
            dn = total - an;
        }
    }
    m->accel_n  = an;
    m->decel_n  = dn;
    m->cruise_n = total - an - dn;
}

/* Instantaneous speed [pps] at pulse_idx within a total_pulses move */
static float GetSpeedAtPulse(uint8_t axis, uint32_t pulse_idx)
{
    MotorCtrl_t *m = &g_motor[axis];
    float s      = (float)pulse_idx + 0.5f;
    float dec_start = (float)m->total_pulses - m->decel_pulses;
    float v_start = ClampSpeed(m->start_pps);
    float v_end   = v_start;
    float accel   = ClampSpeed(m->accel_pps2);
    float speed_sq, speed;

    if (s < m->accel_pulses) {
        speed_sq = v_start * v_start + 2.0f * accel * s;
        speed = SafeSqrt(speed_sq);
    } else if (s < dec_start) {
        speed = m->peak_speed;
    } else {
        float rem = (float)m->total_pulses - s;
        if (rem < 0.0f) rem = 0.0f;
        speed_sq = v_end * v_end + 2.0f * accel * rem;
        speed = SafeSqrt(speed_sq);
    }
    if (speed > m->peak_speed) speed = m->peak_speed;
    return ClampSpeed(speed);
}

/* ── DMA buffer fill helpers ──────────────────────────────────────────────── */

/*
 * Fill a contiguous region of the DMA buffer.
 * No D-Cache maintenance is needed: D-Cache is not enabled in this project,
 * so CPU writes go directly to RAM and DMA always sees up-to-date data.
 */
static void FillDmaBuf(uint8_t axis, uint32_t buf_offset, uint32_t pulse_start,
                        uint32_t count, uint8_t output_on)
{
    uint32_t i;
    for (i = 0u; i < count; i++) {
        uint32_t arr;
        if (output_on) {
            arr = SpeedToArr(GetSpeedAtPulse(axis, pulse_start + i));
        } else {
            arr = SpeedToArr(MOTOR_SPEED_MIN_F);
        }
        motor_dma_buf[axis][buf_offset + i] = arr;
    }
}

/* ── Low-level helpers ────────────────────────────────────────────────────── */

/* CCR は常に ARR 以下でなければ正しいパルスが出ない。
 * ピーク速度時の最小ARRに合わせて固定（ccr ≤ 全ARR を保証）。 */
static void SetPulseCCR(uint8_t axis)
{
    uint32_t peak_arr = SpeedToArr(g_motor[axis].peak_speed);
    uint32_t ccr = peak_arr / 2u;
    if (ccr < 1u) ccr = 1u;
    __HAL_TIM_SET_COMPARE(k_tim[axis], k_ch[axis], ccr);
}

static void MotorHwStop(uint8_t axis)
{
    TIM_HandleTypeDef *htim = k_tim[axis];

    if (htim->hdma[TIM_DMA_ID_UPDATE] != NULL) {
        (void)HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);
    }
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    (void)HAL_TIM_PWM_Stop(htim, k_ch[axis]);   /* CCxE/MOE/CEN 無効化, チャネル状態 READY へ */
    (void)HAL_TIM_Base_Stop(htim);
    htim->Instance->RCR = 0u;

    g_motor[axis].phase = PH_IDLE;
}

/* ISR(EXTI)からの即時停止。動作中DMA(加減速)のAbort(ポーリング)は以前ハングした経路なので、
 * ここでは Abort せずカウンタ/出力/DMA要求を止めるだけにし、DMAストリームの後始末は
 * 次回移動の StartRampBlock 冒頭 DMABurst_WriteStop に委ねる。*/
static void MotorAbortISR(uint8_t axis)
{
    TIM_HandleTypeDef *htim = k_tim[axis];
    __HAL_TIM_DISABLE(htim);                       /* CEN off: パルス即停止 */
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);   /* タイマ→DMA要求停止(非ブロッキング) */
    (void)HAL_TIM_PWM_Stop(htim, k_ch[axis]);      /* 出力無効/チャネル状態 READY */
    htim->Instance->RCR = 0u;
    g_motor[axis].phase = PH_IDLE;
}

/*
 * タイマ出力を起動/再開する。
 * 区間遷移(accel→cruise→decel)やブロック境界では PWM を Stop せず継続するため、
 * 出力(CCxE)とMOEは有効なまま・カウンタ(CEN)だけが StartRampBlock の __HAL_TIM_DISABLE で
 * 止まっている。よって:
 *   - 初回(チャネル READY = 出力停止中): HAL_TIM_PWM_Start で CCxE+MOE+CEN を有効化。
 *   - 継続中(チャネル BUSY = 出力有効): カウンタのみ __HAL_TIM_ENABLE で再投入。
 * これにより未Stopでも HAL_TIM_PWM_Start の BUSY エラーを回避できる。
 */
static int MotorOutputResume(uint8_t axis)
{
    TIM_HandleTypeDef *htim = k_tim[axis];
    uint32_t channel        = k_ch[axis];

    if (TIM_CHANNEL_STATE_GET(htim, channel) == HAL_TIM_CHANNEL_STATE_READY) {
        if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) return -1;  /* CCxE+MOE+CEN */
    } else {
        __HAL_TIM_ENABLE(htim);                                     /* CEN のみ */
    }
    return 0;
}

static void MotorFinish(uint8_t axis)
{
    MotorCtrl_t *m = &g_motor[axis];

    MotorHwStop(axis);

    /* 区間カウントは確定値なので total から正確に位置を確定。
     * ホーミングの原点確定は FSM(MotorStateTick) が担当するためここでは特例にしない
     * （サブ移動が ORG 未検出で自然完了した場合は FSM がエラー検出する）。 */
    m->pos = m->start_pos +
             (m->dir ? (int32_t)m->total_pulses : -(int32_t)m->total_pulses);
    m->state = MOTOR_IDLE;
}

/*
 * 加減速ランプの1ブロックを毎パルス可変ARRのDMAで開始（RCR=0）。
 *  abs_start : 移動内の絶対パルス番号（GetSpeedAtPulse用）
 *  count     : 本ブロックのパルス数（1..BUF）
 *  is_final  : 移動全体の最終ブロック（末尾を低速ホールドで取りこぼし吸収）
 * タイマは毎回 DISABLE→プリロード→UG で起動（ブロック境界に僅少ギャップ）。
 */
static int StartRampBlock(uint8_t axis, uint32_t abs_start, uint32_t count, uint8_t is_final)
{
    TIM_HandleTypeDef *htim = k_tim[axis];
    DMA_HandleTypeDef *hdma = htim->hdma[TIM_DMA_ID_UPDATE];

    if (hdma == NULL || count == 0u || count > MOTOR_DMA_BUF_SIZE) return -1;

    (void)HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);  /* DMABurstState=READYへ（停止済みなら即時）*/
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    htim->Instance->RCR = 0u;                                /* 毎パルスUEV */

    /* 先頭パルスのARRをプリロード */
    uint32_t arr0 = SpeedToArr(GetSpeedAtPulse(axis, abs_start));
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_SET_COUNTER(htim, 0u);
    __HAL_TIM_SET_AUTORELOAD(htim, arr0);
    htim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

    /* buf[0..count-2] = ARR(abs_start+1 .. abs_start+count-1) */
    if (count >= 2u) {
        FillDmaBuf(axis, 0u, abs_start + 1u, count - 1u, 1u);
    }
    /* 末尾エントリ（取りこぼし吸収）: 最終ブロックは低速ホールド、それ以外は次パルスARR（連続）*/
    motor_dma_buf[axis][count - 1u] = is_final
        ? SpeedToArr(MOTOR_SPEED_MIN_F)
        : SpeedToArr(GetSpeedAtPulse(axis, abs_start + count));

    SetPulseCCR(axis);

    if (HAL_TIM_DMABurst_MultiWriteStart(htim, TIM_DMABASE_ARR, TIM_DMA_UPDATE,
            motor_dma_buf[axis], TIM_DMABURSTLENGTH_1TRANSFER, count) != HAL_OK) {
        return -1;
    }
    /* 区間遷移では PWM/Base を Stop していないので、初回のみ出力を有効化し
     * 継続中(BUSY)はカウンタのみ再投入する（PWM_Start の BUSY エラー回避）。*/
    if (MotorOutputResume(axis) != 0) { MotorHwStop(axis); return -1; }

    return 0;
}

/* 現在の区間(ACCEL/CRUISE/DECEL)の次ブロックを供給開始。
 * 巡航も一定ARR(GetSpeedAtPulseがpeakを返す)のDMAブロックで駆動する。*/
static int StartRampSegment(uint8_t axis)
{
    MotorCtrl_t *m = &g_motor[axis];
    uint32_t phase_total, base;
    if      (m->phase == PH_ACCEL)  { phase_total = m->accel_n;  base = 0u; }
    else if (m->phase == PH_CRUISE) { phase_total = m->cruise_n; base = m->accel_n; }
    else                            { phase_total = m->decel_n;  base = m->accel_n + m->cruise_n; }

    uint32_t remain = phase_total - m->seg_issued;
    uint32_t count  = (remain > MOTOR_DMA_BUF_SIZE) ? MOTOR_DMA_BUF_SIZE : remain;
    uint8_t  last   = (m->seg_issued + count) >= phase_total;
    /* 末尾低速ホールドは移動全体の最終ブロックのみ（DECEL最終、または DECELなしのCRUISE最終）*/
    uint8_t  is_final = last && ((m->phase == PH_DECEL) ||
                                 (m->phase == PH_CRUISE && m->decel_n == 0u));

    if (count == 0u) return -1;
    if (StartRampBlock(axis, base + m->seg_issued, count, is_final) != 0) return -1;
    m->seg_issued += count;
    return 0;
}

static void EnterDecel(uint8_t axis);

/* CRUISE開始: 一定ARR(peak)で「連続運転」（タイマを止めない）。RCRで CRUISE_CHUNK パルス毎にUEV割込。
 * DMA化すると境界ごとにタイマ停止→再起動で脱調するため、巡航はRCR連続運転にする。
 * 手動停止は stop_req により現チャンク完了時(UEV)に止めるので計数誤差はゼロ。*/
static void EnterCruise(uint8_t axis)
{
    MotorCtrl_t *m = &g_motor[axis];
    TIM_HandleTypeDef *htim = k_tim[axis];

    if (m->cruise_n == 0u) { EnterDecel(axis); return; }

    /* 加速DMAの後始末（加速最終TCで停止済み＝即時）。タイマ自体は回したまま（ギャップ無し）*/
    (void)HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);

    __HAL_TIM_SET_AUTORELOAD(htim, SpeedToArr(m->peak_speed));  /* 一定ARR(peak) */

    m->phase         = PH_CRUISE;
    m->cruise_remain = m->cruise_n;

    uint32_t chunk = (m->cruise_remain > CRUISE_CHUNK) ? CRUISE_CHUNK : m->cruise_remain;
    htim->Instance->RCR = chunk - 1u;   /* 次UEVから chunk パルス毎に割込 */
    m->cruise_remain -= chunk;

    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
}

/* DECEL開始 */
static void EnterDecel(uint8_t axis)
{
    MotorCtrl_t *m = &g_motor[axis];

    __HAL_TIM_DISABLE_IT(k_tim[axis], TIM_IT_UPDATE);

    if (m->decel_n == 0u) { MotorFinish(axis); return; }

    m->phase      = PH_DECEL;
    m->seg_issued = 0u;
    if (StartRampSegment(axis) != 0) { MotorHwStop(axis); m->state = MOTOR_IDLE; }
}

/* 移動開始（ACCEL→CRUISE→DECEL の起点） */
static int StartMotionDma(uint8_t axis)
{
    MotorCtrl_t *m = &g_motor[axis];
    TIM_HandleTypeDef *htim = k_tim[axis];

    if (m->total_pulses == 0u) return -1;

    if (m->dir) { CVD_SetDirCW(g_cvd[axis]); } else { CVD_SetDirCCW(g_cvd[axis]); }
    m->start_pos  = m->pos;
    m->seg_issued = 0u;

    if (m->accel_n > 0u) {
        m->phase = PH_ACCEL;
        return StartRampSegment(axis);
    }

    if (m->cruise_n > 0u) {
        /* 加速なし: タイマをpeakで直接起動してから巡航(連続RCR)へ */
        __HAL_TIM_DISABLE(htim);
        __HAL_TIM_SET_COUNTER(htim, 0u);
        __HAL_TIM_SET_AUTORELOAD(htim, SpeedToArr(m->peak_speed));
        htim->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
        SetPulseCCR(axis);
        if (MotorOutputResume(axis) != 0) { MotorHwStop(axis); return -1; }
        EnterCruise(axis);
        return 0;
    }

    /* 減速のみ（極小移動）*/
    m->phase = PH_DECEL;
    return StartRampSegment(axis);
}

/* ── フェーズエンジン（UEV起点の状態遷移） ───────────────────────────────────
 * 呼び出し元（いずれも HAL_TIM_PeriodElapsedCallback 経由）:
 *  - ACCEL/DECEL中: DMAブロック完了TC（DMAストリーム停止済み＝再起動安全）
 *  - CRUISE中     : TIM_UP割込（連続運転・DMA非動作。RCRで chunk パルス毎）
 */
void Motor_OnUpdateEvent(TIM_HandleTypeDef *htim)
{
    uint8_t axis;
    MotorCtrl_t *m;

    if (!FindAxisByTim(htim, &axis)) return;
    m = &g_motor[axis];

    switch (m->phase) {
    case PH_ACCEL:
        if (m->seg_issued >= m->accel_n) {
            EnterCruise(axis);                 /* cruise_n==0 なら内部で DECEL */
        } else {
            if (StartRampSegment(axis) != 0) { MotorHwStop(axis); m->state = MOTOR_IDLE; }
        }
        break;

    case PH_CRUISE:
        if (m->stop_req) {
            /* 手動停止要求: 急停止せず、現チャンク完了(=この境界, cruise_done は厳密)から
             * 事前計算済みの減速 decel_n(peak→v_end) を実行して滑らかに止める（オーバーシュート低減）。
             * 巡航をここまでに短縮し total を再定義 → 減速後 MotorFinish が pos を厳密確定。*/
            m->stop_req = 0u;
            uint32_t cruise_done = m->cruise_n - m->cruise_remain;
            m->cruise_n     = cruise_done;
            m->total_pulses = m->accel_n + cruise_done + m->decel_n;
            EnterDecel(axis);   /* decel_n==0 なら内部で MotorFinish */
        } else if (m->cruise_remain > 0u) {
            uint32_t chunk = (m->cruise_remain > CRUISE_CHUNK) ? CRUISE_CHUNK : m->cruise_remain;
            htim->Instance->RCR = chunk - 1u;  /* 次UEVから chunk パルス */
            m->cruise_remain -= chunk;
        } else {
            EnterDecel(axis);
        }
        break;

    case PH_DECEL:
        if (m->seg_issued >= m->decel_n) {
            MotorFinish(axis);
        } else {
            if (StartRampSegment(axis) != 0) { MotorHwStop(axis); m->state = MOTOR_IDLE; }
        }
        break;

    default:
        break;
    }
}

/*
 * HAL weak-function override（全タイマの更新イベントがここに集約される）。
 *  - TIM6: 10ms 状態ティック → MotorStateTick()
 *  - TIM1/8/3: ACCEL/DECEL の DMA完了(TIM_DMAPeriodElapsedCplt経由) と
 *              CRUISE の UEV(TIMx_UP割込→HAL_TIM_IRQHandler経由) → Motor_OnUpdateEvent()
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        MotorStateTick();
        return;
    }
    Motor_OnUpdateEvent(htim);
}
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;  /* 未使用（DMAハーフ転送割込は無視） */
}

/* ── Public motor API ─────────────────────────────────────────────────────── */

static void MotorPrepare(uint8_t axis, MotorState_t new_state,
                          uint8_t dir, uint32_t total, float speed)
{
    MotorCtrl_t *m = &g_motor[axis];
    int32_t saved_pos = m->pos;          /* pos は移動間で保持する */

    memset(m, 0, sizeof(MotorCtrl_t));
    m->pos           = saved_pos;
    m->state         = new_state;
    m->dir           = dir;
    m->total_pulses  = total;
    m->phase         = PH_IDLE;

    /* 速度を安全上限でクランプ（誤設定で 500kHz 等を要求されても抑止） */
    if (speed > (float)MOTOR_MAX_PPS) speed = (float)MOTOR_MAX_PPS;

    /* 初速度・加速度を軸別パラメータから決定（加速度 = (目標−初速) / 加速時間） */
    m->start_pps = (float)P_start_pps[axis];
    {
        float t_s = (float)P_accel_ms[axis] / 1000.0f;
        if (t_s < 0.001f) t_s = 0.001f;            /* 0除算防止 */
        m->accel_pps2 = ((float)speed - m->start_pps) / t_s;
        if (m->accel_pps2 < 1.0f) m->accel_pps2 = 1.0f;
    }

    BuildMotionProfile(axis, total, speed);
}

static void MotorMoveRel(uint8_t axis, int32_t pulses, uint8_t wait)
{
    uint8_t dir;
    uint32_t total;

    if (g_motor[axis].state != MOTOR_IDLE || pulses == 0) return;

    dir   = (pulses > 0) ? 1u : 0u;
    total = (uint32_t)((pulses > 0) ? pulses : -pulses);

    /* ハードリミット: 進行方向の端センサが既に作動中なら移動しない
     * （端に張り付いた状態では EXTI エッジが出ないため、ここで弾く）。*/
    if (!dir && CcwActive(axis)) return;               /* CCW(−端) */
#if LIMIT_CW_ENABLE
    if (dir && CwActive(axis)) return;                 /* CW(+端) */
#endif

    /* Clamp to software limits（ホーミング後のみ。未ホーミング時は位置不定のため非適用）。
     * 余裕(room)を符号付きで求め、total を減らす方向にのみクランプする。
     * （旧実装は P_limit が pos の反対側にあると (uint32_t) 変換でアンダーフローし
     *   total が巨大化＝暴走していた。room<=0 は移動不可として弾く。）*/
    if (g_homed[axis]) {
        int32_t pos  = g_motor[axis].pos;
        int32_t room = dir ? (P_limit_cw[axis] - pos)      /* CW : 上限までの余裕 */
                           : (pos - P_limit_ccw[axis]);    /* CCW: 下限までの余裕 */
        if (room <= 0) return;                             /* 余裕なし＝移動しない */
        if (total > (uint32_t)room) total = (uint32_t)room;
    }

    MotorPrepare(axis, MOTOR_MOVING, dir, total, (float)g_speed_pps);
    if (StartMotionDma(axis) != 0) {
        g_motor[axis].state = MOTOR_IDLE;
        return;
    }

    if (wait) {
        while (g_motor[axis].state != MOTOR_IDLE) {}
    }
}

static void MotorMoveAbs(uint8_t axis, int32_t target, uint8_t wait)
{
    MotorMoveRel(axis, target - g_motor[axis].pos, wait);
}

/* 一定速(cruise-only, DMA非動作)で移動を開始する。homing のサブ移動で使用。
 * EXTI からの即停止が安全（動作中DMAのAbortが無い）。total は保険上限。*/
static int MotorStartConstant(uint8_t axis, uint8_t dir, uint32_t total, uint32_t speed)
{
    MotorCtrl_t *m = &g_motor[axis];
    int32_t saved_pos = m->pos;

    memset(m, 0, sizeof(MotorCtrl_t));
    m->pos          = saved_pos;
    m->state        = MOTOR_HOMING;
    m->dir          = dir;
    m->total_pulses = total;
    m->phase        = PH_IDLE;

    float sp = (float)speed;
    if (sp > (float)MOTOR_MAX_PPS) sp = (float)MOTOR_MAX_PPS;
    if (sp < MOTOR_SPEED_MIN_F)    sp = MOTOR_SPEED_MIN_F;
    m->peak_speed = sp;
    m->start_pps  = sp;
    m->accel_pps2 = 1.0f;
    /* cruise-only: 加減速なし */
    m->accel_n = 0u;
    m->decel_n = 0u;
    m->cruise_n = total;

    return StartMotionDma(axis);
}

/* 3段ホーミング開始: FSM を起動。実際の進行は MotorStateTick(10ms) + EXTI で行う。 */
static void MotorHome(uint8_t axis, uint8_t wait)
{
    if (g_motor[axis].state != MOTOR_IDLE) return;

    g_home_evt[axis] = HEVT_NONE;

    if (CcwActive(axis)) {
        /* 既に CCW limit 上 → バックオフから開始 */
        g_homing[axis] = HM_BACKOFF;
        if (MotorStartConstant(axis, (uint8_t)(!HOME_SEEK_DIR),
                               (uint32_t)HOME_MAX_TRAVEL, HOME_SLOW_PPS) != 0) {
            g_homing[axis] = HM_ERROR; g_motor[axis].state = MOTOR_IDLE; return;
        }
    } else {
        g_homing[axis] = HM_FAST_SEEK;
        if (MotorStartConstant(axis, HOME_SEEK_DIR,
                               (uint32_t)HOME_MAX_TRAVEL, HOME_FAST_PPS) != 0) {
            g_homing[axis] = HM_ERROR; g_motor[axis].state = MOTOR_IDLE; return;
        }
    }

    if (wait) {
        while (g_homing[axis] != HM_DONE && g_homing[axis] != HM_ERROR) {}
    }
}

static void MotorJog(uint8_t axis, uint8_t positive)
{
    uint32_t range;
    int32_t  limit;

    if (g_motor[axis].state != MOTOR_IDLE) return;

    /* ハードリミット: 進行方向の端センサが既に作動中なら移動しない */
    if (!positive && CcwActive(axis)) return;          /* CCW(−端) */
#if LIMIT_CW_ENABLE
    if (positive && CwActive(axis)) return;            /* CW(+端) */
#endif

    if (!g_homed[axis]) {
        /* 未ホーミング: 位置不定のためリミット非適用。十分大きい移動量で起動し
         * 停止は MotorJogStop（手動）/ ハード端センサ(EXTI)で行う。*/
        range = (uint32_t)HOME_MAX_TRAVEL;
    } else if (positive) {
        limit = P_limit_cw[axis];
        range = (uint32_t)(limit - g_motor[axis].pos);
    } else {
        limit = P_limit_ccw[axis];
        range = (uint32_t)(g_motor[axis].pos - limit);
    }
    if (range == 0u) return;

    MotorPrepare(axis, MOTOR_JOGGING, positive ? 1u : 0u,
                 range, (float)g_speed_pps);
    if (StartMotionDma(axis) != 0) {
        g_motor[axis].state = MOTOR_IDLE;
    }
}

/* 現在の出力済みパルス数を求める（加減速=DMA NDTRで厳密／巡航=cruise_remainから, 誤差≤CRUISE_CHUNK）。
 * NDTR/レジスタ読みのみなので ISR/メインどちらからでも可。*/
static uint32_t MotorEmittedNow(uint8_t axis)
{
    MotorCtrl_t *m = &g_motor[axis];
    DMA_HandleTypeDef *hdma = k_tim[axis]->hdma[TIM_DMA_ID_UPDATE];
    uint32_t ndtr     = (hdma != NULL) ? __HAL_DMA_GET_COUNTER(hdma) : 0u;
    uint32_t seg_done = (m->seg_issued > ndtr) ? (m->seg_issued - ndtr) : 0u;
    uint32_t emitted;
    switch (m->phase) {
    case PH_ACCEL:  emitted = seg_done; break;                                      /* DMA: NDTR厳密 */
    case PH_DECEL:  emitted = m->accel_n + m->cruise_n + seg_done; break;           /* DMA: NDTR厳密 */
    case PH_CRUISE: emitted = m->accel_n + (m->cruise_n - m->cruise_remain); break; /* RCR: ≤CRUISE_CHUNK誤差 */
    default:        emitted = 0u; break;
    }
    if (emitted > m->total_pulses) emitted = m->total_pulses;
    return emitted;
}

static void MotorJogStop(uint8_t axis)
{
    MotorCtrl_t *m = &g_motor[axis];

    if (m->state != MOTOR_JOGGING && m->state != MOTOR_MOVING) return;

    /* 巡航中は即停止せず、実行中のRCRチャンクを出し切ってから境界(UEV)で停止する
     * （その時点で cruise_n-cruise_remain が実出力と一致＝計数誤差ゼロ）。
     * phase 読取〜フラグ設定を割込禁止で行い、巡航UEV との競合を防ぐ。*/
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (m->phase == PH_CRUISE) {
        m->stop_req = 1u;
        __set_PRIMASK(primask);
        return;
    }
    __set_PRIMASK(primask);

    /* 加減速中(または既にIDLE寸前): NDTR で厳密に求めて即停止 */
    uint32_t emitted = MotorEmittedNow(axis);
    MotorHwStop(axis);
    m->pos   = m->start_pos + (m->dir ? (int32_t)emitted : -(int32_t)emitted);
    m->state = MOTOR_IDLE;
}

/* ── EXTI: センサ即停止（精密）。ピン番号は全センサで一意なので軸/種別を識別可能。 ── */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    for (uint8_t a = 0u; a < MOTOR_MAX; a++) {
        HomingState_t hs = g_homing[a];

        if (hs == HM_FAST_SEEK || hs == HM_BACKOFF || hs == HM_SLOW_SEEK) {
            /* ── ホーミング中 ─────────────────────────────────────────── */
            /* CCW limit = 原点基準センサ */
            if (pin == k_ccw_pin[a]) {
                if (hs == HM_BACKOFF) {
                    if (!CcwActive(a)) { MotorAbortISR(a); g_home_evt[a] = HEVT_HOME; }  /* 解除 */
                } else { /* FAST_SEEK / SLOW_SEEK */
                    if (CcwActive(a)) { MotorAbortISR(a); g_home_evt[a] = HEVT_HOME; }   /* 作動 */
                }
                return;
            }
#if LIMIT_CW_ENABLE
            /* CW limit = 過走防止（異常） */
            if (pin == k_cw_pin[a]) {
                if (CwActive(a)) { MotorAbortISR(a); g_home_evt[a] = HEVT_LIMIT; }
                return;
            }
#endif
            /* ORG はホーミングに不使用（無視） */
        } else {
            /* ── 通常移動中(MOVING/JOGGING)のハードリミット即停止 ────────
             * 進行方向の端センサが作動したら停止（逆方向には逃げられる）。*/
            MotorCtrl_t *m = &g_motor[a];
            if (m->state == MOTOR_MOVING || m->state == MOTOR_JOGGING) {
                if ((pin == k_ccw_pin[a]) && (m->dir == 0u) && CcwActive(a)) {
                    /* CCW端で即停止。位置は実出力パルスから確定する（原点0へのリセットはしない）。
                     * ISR安全停止（動作中DMAのAbortを避ける）。*/
                    uint32_t emitted = MotorEmittedNow(a);
                    MotorAbortISR(a);
                    m->pos   = m->start_pos - (int32_t)emitted;   /* CCW(負)方向 */
                    m->state = MOTOR_IDLE;
                    return;
                }
#if LIMIT_CW_ENABLE
                if ((pin == k_cw_pin[a])  && (m->dir != 0u) && CwActive(a)) {
                    MotorAbortISR(a); g_motor[a].state = MOTOR_IDLE;  /* CW有効時: 位置更新は要精密化 */
                    return;
                }
#endif
            }
        }
    }
}

/* ── ホーミング FSM 進行（TIM6 10ms から呼ぶ） ── */
void MotorStateTick(void)
{
    for (uint8_t a = 0u; a < MOTOR_MAX; a++) {
        HomingState_t hs = g_homing[a];

        /* 原点への CW オフセット移動が自然完了(IDLE)したら原点確定 */
        if (hs == HM_OFFSET) {
            if (g_motor[a].state == MOTOR_IDLE) {   /* offset移動完了 → MotorFinishで pos=0 */
                g_homing[a] = HM_DONE;
                g_homed[a]  = 1u;
            }
            continue;
        }

        if (hs != HM_FAST_SEEK && hs != HM_BACKOFF && hs != HM_SLOW_SEEK) continue;

        HomeEvt_t evt = g_home_evt[a];

        /* CW 端で停止 = 過走防止（異常） */
        if (evt == HEVT_LIMIT) {
            g_home_evt[a] = HEVT_NONE;
            MotorHwStop(a);
            g_homing[a] = HM_ERROR;
            g_motor[a].state = MOTOR_IDLE;
            continue;
        }

        /* CCW limit エッジで停止 → 次サブ移動へ */
        if (evt == HEVT_HOME) {
            g_home_evt[a] = HEVT_NONE;
            MotorHwStop(a);   /* EXTIで停止済みだが状態を確実に */
            switch (hs) {
            case HM_FAST_SEEK:   /* CCW作動で停止 → CCWから離れる */
                g_homing[a] = HM_BACKOFF;
                if (MotorStartConstant(a, (uint8_t)(!HOME_SEEK_DIR),
                        (uint32_t)HOME_MAX_TRAVEL, HOME_SLOW_PPS) != 0) {
                    g_homing[a] = HM_ERROR; g_motor[a].state = MOTOR_IDLE;
                }
                break;
            case HM_BACKOFF:     /* CCW解除で停止 → 低速で再接近 */
                g_homing[a] = HM_SLOW_SEEK;
                if (MotorStartConstant(a, HOME_SEEK_DIR,
                        (uint32_t)HOME_MAX_TRAVEL, HOME_SLOW_PPS) != 0) {
                    g_homing[a] = HM_ERROR; g_motor[a].state = MOTOR_IDLE;
                }
                break;
            case HM_SLOW_SEEK:   /* CCW作動で停止 → 原点基準を確定 */
                if (P_home_offset[a] > 0) {
                    /* 原点(0)を CCWセンサから CW側へ offset だけずらす:
                     * センサ位置を -offset とし、CWへ offset パルス「通常移動(加減速, g_speed_pps)」
                     * して pos=0 に停止。MotorMoveRel は state!=IDLE だと弾くので先に IDLE 化。
                     * ホーミング未完(g_homed=0)なのでソフトリミットは掛からず offset 全量動く。*/
                    g_motor[a].pos   = -P_home_offset[a];
                    g_motor[a].state = MOTOR_IDLE;
                    g_homing[a]      = HM_OFFSET;
                    MotorMoveRel(a, P_home_offset[a], 0u);   /* +offset = CW方向 */
                    if (g_motor[a].state != MOTOR_MOVING) { g_homing[a] = HM_ERROR; }  /* 起動失敗 */
                } else {                      /* オフセットなし: センサ位置=原点0（従来）*/
                    g_motor[a].pos   = 0;
                    g_homing[a]      = HM_DONE;
                    g_homed[a]       = 1u;   /* 以後ソフトリミット適用 */
                    g_motor[a].state = MOTOR_IDLE;
                }
                break;
            default: break;
            }
            continue;
        }

        /* イベント無しでサブ移動が自然終了(IDLE) = CCW/CW 未検出 → 異常 */
        if (g_motor[a].state == MOTOR_IDLE) {
            g_homing[a] = HM_ERROR;
        }
    }
}

/* ── Command parser ───────────────────────────────────────────────────────── */

#define RESP_SZ  64u

static int ParseSubCmd(const char *tok, char *resp)
{
    uint8_t axis;
    char    cmd;

    if (tok == NULL || tok[0] == '\0') return 0;

    /* MS / mS speed */
    if (tok[0] == 'M' && tok[1] == 'S') {
        if (tok[2] == 'R') { snprintf(resp, RESP_SZ, "%lu", (unsigned long)g_speed_pps); return 1; }
        g_speed_pps = (uint32_t)atoi(tok + 2);
        return 0;
    }
    if (tok[0] == 'm' && tok[1] == 'S') {
        if (tok[2] == 'R') { snprintf(resp, RESP_SZ, "%.6g", (double)g_speed_mm); return 1; }
        g_speed_mm = (float)atof(tok + 2);
        return 0;
    }

    /* Register commands */
    if (tok[0] == 'R') {
        if (tok[1] == 'S') {
            if (Parm_Flash_Save() == HAL_OK) snprintf(resp, RESP_SZ, "OK");
            else                             snprintf(resp, RESP_SZ, "NG");
            return 1;
        }
        if (tok[1] == 'A') {
            char line[64];
            for (int i = 0; i < REG_SIZE; i++) {
                if (parm[i].unit == UNIT_FLOAT)
                    snprintf(line, sizeof(line), "%d:%.11s:%d:%.6g\n",
                             i, parm[i].name, (int)parm[i].unit, (double)parm[i].data_f);
                else
                    snprintf(line, sizeof(line), "%d:%.11s:%d:%ld\n",
                             i, parm[i].name, (int)parm[i].unit, (long)parm[i].data);
                CDC_SendString(line);
            }
            snprintf(resp, RESP_SZ, "END"); return 1;
        }
        if (tok[1] == 'P') {   /* Re-apply: parm[] を P_*（リミット等）へ即時反映。
                                * Parm_set() は起動時のみ実行のため、変更を再起動なしで効かせる。
                                * Motor_Config(CVD再設定/励磁)も走るので全軸IDLE時のみ許可。*/
            for (uint8_t i = 0u; i < MOTOR_MAX; i++) {
                if (g_motor[i].state != MOTOR_IDLE) { snprintf(resp, RESP_SZ, "BUSY"); return 1; }
            }
            Parm_set();
            snprintf(resp, RESP_SZ, "OK");
            return 1;
        }
        {
            int addr = atoi(tok + 1);
            if (addr < 0 || addr >= REG_SIZE) { snprintf(resp, RESP_SZ, "NG"); return 1; }
            const char *p = tok + 1;
            while (*p >= '0' && *p <= '9') p++;
            if (*p == 'S') {
                if (parm[addr].unit == UNIT_FLOAT) parm[addr].data_f = (float)atof(p + 1);
                else                               parm[addr].data   = (int32_t)atoi(p + 1);
                snprintf(resp, RESP_SZ, "OK");
            } else if (*p == 'R') {
                if (parm[addr].unit == UNIT_FLOAT) snprintf(resp, RESP_SZ, "%.6g", (double)parm[addr].data_f);
                else                               snprintf(resp, RESP_SZ, "%ld",  (long)parm[addr].data);
            } else { snprintf(resp, RESP_SZ, "NG"); }
            return 1;
        }
    }

    /* Pulse-mode M<n>... */
    if (tok[0] == 'M' && tok[1] >= '0' && tok[1] <= '2') {
        axis = (uint8_t)(tok[1] - '0');
        cmd  = tok[2];
        switch (cmd) {
            case 'R': MotorMoveRel(axis, atoi(tok+3), 0); snprintf(resp, RESP_SZ, "OK"); break;
            case 'r': MotorMoveRel(axis, atoi(tok+3), 1); snprintf(resp, RESP_SZ, "OK"); break;
            case 'A': MotorMoveAbs(axis, atoi(tok+3), 0); snprintf(resp, RESP_SZ, "OK"); break;
            case 'a': MotorMoveAbs(axis, atoi(tok+3), 1); snprintf(resp, RESP_SZ, "OK"); break;
            case 'P': snprintf(resp, RESP_SZ, "%ld", (long)g_motor[axis].pos); break;
            case 'L': {  /* センサ診断: 論理active(検出=1) と 生ピンレベル(High=1) */
                uint8_t rorg = (HAL_GPIO_ReadPin(k_org_port[axis], k_org_pin[axis]) == GPIO_PIN_SET) ? 1u : 0u;
                uint8_t rcw  = (HAL_GPIO_ReadPin(k_cw_port[axis],  k_cw_pin[axis])  == GPIO_PIN_SET) ? 1u : 0u;
                uint8_t rccw = (HAL_GPIO_ReadPin(k_ccw_port[axis], k_ccw_pin[axis]) == GPIO_PIN_SET) ? 1u : 0u;
                snprintf(resp, RESP_SZ, "ORG=%u CW=%u CCW=%u rawH ORG=%u CW=%u CCW=%u",
                         OrgActive(axis), CwActive(axis), CcwActive(axis), rorg, rcw, rccw);
                break;
            }
            case 'I': MotorHome(axis, 0); snprintf(resp, RESP_SZ, "OK"); break;
            case 'i': MotorHome(axis, 1); snprintf(resp, RESP_SZ, "OK"); break;
            case 'N':
                if (tok[3]=='P') MotorJog(axis, 1u);
                else             MotorJog(axis, 0u);
                snprintf(resp, RESP_SZ, "OK"); break;
            case 'F': MotorJogStop(axis); snprintf(resp, RESP_SZ, "OK"); break;
            default:  snprintf(resp, RESP_SZ, "NG"); break;
        }
        return 1;
    }

    /* Physical-mode m<n>... */
    if (tok[0] == 'm' && tok[1] >= '0' && tok[1] <= '2') {
        axis = (uint8_t)(tok[1] - '0');
        cmd  = tok[2];
        float coeff = P_motor_coeff[axis];
        if (coeff == 0.0f) { snprintf(resp, RESP_SZ, "NG"); return 1; }

        uint32_t saved = g_speed_pps;
        g_speed_pps = (uint32_t)(g_speed_mm / coeff);
        if (g_speed_pps < 1u) g_speed_pps = 1u;

        switch (cmd) {
            case 'R': MotorMoveRel(axis, (int32_t)(atof(tok+3)/coeff), 0); snprintf(resp,RESP_SZ,"OK"); break;
            case 'r': MotorMoveRel(axis, (int32_t)(atof(tok+3)/coeff), 1); snprintf(resp,RESP_SZ,"OK"); break;
            case 'A': MotorMoveAbs(axis, (int32_t)(atof(tok+3)/coeff), 0); snprintf(resp,RESP_SZ,"OK"); break;
            case 'a': MotorMoveAbs(axis, (int32_t)(atof(tok+3)/coeff), 1); snprintf(resp,RESP_SZ,"OK"); break;
            case 'P': snprintf(resp,RESP_SZ,"%.6g",(double)((float)g_motor[axis].pos*coeff)); break;
            case 'I': MotorHome(axis,0); snprintf(resp,RESP_SZ,"OK"); break;
            case 'i': MotorHome(axis,1); snprintf(resp,RESP_SZ,"OK"); break;
            case 'N':
                if (tok[3]=='P') MotorJog(axis,1u);
                else             MotorJog(axis,0u);
                snprintf(resp,RESP_SZ,"OK"); break;
            case 'F': MotorJogStop(axis); snprintf(resp,RESP_SZ,"OK"); break;
            default:  snprintf(resp,RESP_SZ,"NG"); break;
        }
        g_speed_pps = saved;
        return 1;
    }

    /* ── 汎用 GPIO 出力 — prefix "G" ──
     * G<n>S<0|1> : GPO n(0..3=PE9..PE12) を物理 Low(0)/High(1) に設定（応答 OK）
     * G<n>R      : 物理レベル(0/1) を返す
     * 極性(High=ON / Low=ON)適用は GUI 側。ここは生レベルのみ扱う。 */
    if (tok[0] == 'G' && tok[1] >= '0' && tok[1] <= '3') {
        uint8_t n = (uint8_t)(tok[1] - '0');
        cmd = tok[2];
        if (cmd == 'S') {
            GPIO_PinState lv = (tok[3] == '1') ? GPIO_PIN_SET : GPIO_PIN_RESET;
            HAL_GPIO_WritePin(k_gpo_port[n], k_gpo_pin[n], lv);
            snprintf(resp, RESP_SZ, "OK");
        } else if (cmd == 'R') {
            uint8_t lv = (HAL_GPIO_ReadPin(k_gpo_port[n], k_gpo_pin[n]) == GPIO_PIN_SET) ? 1u : 0u;
            snprintf(resp, RESP_SZ, "%u", lv);
        } else {
            snprintf(resp, RESP_SZ, "NG");
        }
        return 1;
    }

    /* ── MTD415T TEC temperature controller (USART3 bridge) — prefix "TC" ──
     * Host-facing temperatures are in °C (float); the device uses m°C natively.
     * TC$<raw> forwards any raw MTD415T command verbatim (full command access). */
    if (tok[0] == 'T' && tok[1] == 'C') {
        const char *p = tok + 2;
        char    dev[40];
        int32_t v32;
        switch (p[0]) {
            case '$':   /* raw pass-through: TC$<raw>  (e.g. TC$Px100, TC$E?) */
                if (strchr(p + 1, '?') != NULL) {
                    if (MTD415T_Query(p + 1, dev, sizeof(dev), MTD415T_QUERY_TIMEOUT_MS) < 0)
                        snprintf(resp, RESP_SZ, "NG");
                    else
                        snprintf(resp, RESP_SZ, "%s", dev);
                } else {
                    MTD415T_SendRaw(p + 1);
                    snprintf(resp, RESP_SZ, "OK");
                }
                break;
            case 'T':   /* TCT<°C> set setpoint / TCTR read setpoint */
                if (p[1] == 'R') {
                    if (MTD415T_GetSetTempMdegC(&v32) < 0) snprintf(resp, RESP_SZ, "NG");
                    else snprintf(resp, RESP_SZ, "%.3f", (double)v32 / 1000.0);
                } else {
                    MTD415T_SetTempMdegC((int32_t)lroundf((float)atof(p + 1) * 1000.0f));
                    snprintf(resp, RESP_SZ, "OK");
                }
                break;
            case 'M':   /* TCM read measured temperature [°C] */
                if (MTD415T_GetActualTempMdegC(&v32) < 0) snprintf(resp, RESP_SZ, "NG");
                else snprintf(resp, RESP_SZ, "%.3f", (double)v32 / 1000.0);
                break;
            case 'A':   /* TCA read TEC current [mA] */
                if (MTD415T_GetCurrentmA(&v32) < 0) snprintf(resp, RESP_SZ, "NG");
                else snprintf(resp, RESP_SZ, "%ld", (long)v32);
                break;
            case 'L':   /* TCL<mA> set TEC current limit */
                MTD415T_SetCurrentLimitmA((int32_t)atoi(p + 1));
                snprintf(resp, RESP_SZ, "OK");
                break;
            case 'E':   /* TCE read error register */
                { uint32_t e;
                  if (MTD415T_GetError(&e) < 0) snprintf(resp, RESP_SZ, "NG");
                  else snprintf(resp, RESP_SZ, "%lu", (unsigned long)e); }
                break;
            case 'V':   /* TCV read version */
                if (MTD415T_GetVersion(dev, sizeof(dev)) < 0) snprintf(resp, RESP_SZ, "NG");
                else snprintf(resp, RESP_SZ, "%s", dev);
                break;
            case 'S':   /* TCS save settings to MTD415T flash */
                MTD415T_Save();
                snprintf(resp, RESP_SZ, "OK");
                break;
            default:
                snprintf(resp, RESP_SZ, "NG");
                break;
        }
        return 1;
    }

    snprintf(resp, RESP_SZ, "NG");
    return 1;
}

/* カスケード送信（マスター→スレーブ）。step3 で実装。0=成功, 非0=失敗 */
static int Cascade_Send(uint8_t box, const char *cmd, char *resp, size_t n);

/*
 * RunCommandLine — 1行（; 区切り可）を実行し、応答文字列を resp_out に生成する。
 * ルーティングプレフィックス（@<box>;）は含めないローカルコマンド列を渡すこと。
 * 応答が空なら "OK" を返す。CDC/カスケード両経路から再利用する共通実行部。
 */
void RunCommandLine(const char *line, char *resp_out, size_t n)
{
    char  buf[128];
    char  resp[RESP_SZ];
    char *tok;

    strncpy(buf, line, sizeof(buf) - 1u);
    buf[sizeof(buf) - 1u] = '\0';
    resp[0] = '\0';

    tok = strtok(buf, ";");
    while (tok != NULL) {
        ParseSubCmd(tok, resp);
        tok = strtok(NULL, ";");
    }

    if (resp[0] == '\0') snprintf(resp_out, n, "OK");
    else                 snprintf(resp_out, n, "%s", resp);
}

static void Cmd_Execute(const char *line)
{
    char resp[RESP_SZ], out[RESP_SZ + 2u];

    /* ルーティング: 先頭が @<box>; なら対象機を判定して振り分け */
    if (line[0] == '@') {
        int         box  = atoi(line + 1);
        const char *semi = strchr(line, ';');
        const char *rest = (semi != NULL) ? semi + 1 : "";
        if ((uint8_t)box == P_box_no) {
            RunCommandLine(rest, resp, sizeof(resp));           /* 自機宛 */
        } else if (P_is_master) {
            if (Cascade_Send((uint8_t)box, rest, resp, sizeof(resp)) != 0) {
                snprintf(resp, sizeof(resp), "NG");             /* 転送失敗/タイムアウト */
            }
        } else {
            snprintf(resp, sizeof(resp), "NG");                 /* スレーブが他機宛を受信(想定外) */
        }
    } else {
        RunCommandLine(line, resp, sizeof(resp));               /* 従来互換=自機ローカル */
    }

    snprintf(out, sizeof(out), "%s\n", resp);
    CDC_SendString(out);
}

void Cmd_Process(void)
{
    static char    cmd_line[128];
    static uint8_t cmd_len = 0u;
    uint8_t b;

    while (CDC_RxReadByte(&b)) {
        if (b == (uint8_t)'\r') continue;
        if (b == (uint8_t)'\n') {
            if (cmd_len > 0u) {
                cmd_line[cmd_len] = '\0';
                Cmd_Execute(cmd_line);
                cmd_len = 0u;
            }
        } else if (cmd_len < (uint8_t)(sizeof(cmd_line) - 1u)) {
            cmd_line[cmd_len++] = (char)b;
        }
    }
    g_usb_rx_ready = 0u;
}

/* ── カスケード接続（コントローラ間 SPI, hspi1: PB3/PB4/PB5） ──────────────────
 *
 * マスター機(SPI マスター)が PC から受けた他機宛コマンドを、スレーブ機(SPI スレーブ)へ
 * 中継し応答を返す。CS は使わず、フレーム内 BOX でスレーブを選択（初期は2台=単一スレーブ）。
 *
 * フレーム(固定長 CASC_FRAME_SZ):
 *   [0]=SYNC(0xA5) [1]=TYPE [2]=BOX [3]=LEN [4..]=PAYLOAD(≤128) [末尾]=CRC8
 *   TYPE: CMD(要求) / POLL(応答要求) / BUSY(未完) / READY(応答)
 *
 * 手順: マスターが CMD 送信 → スレーブが主ループで実行し応答を用意 →
 *       マスターが POLL を繰り返し READY を受信するまで再試行（CRC 検証）。
 * ─────────────────────────────────────────────────────────────────────────── */

#define CASC_SYNC          0xA5u
#define CASC_PAYLOAD_MAX   128u
#define CASC_FRAME_SZ      (4u + CASC_PAYLOAD_MAX + 1u)   /* = 133 */
#define CASC_T_CMD         0x01u
#define CASC_T_POLL        0x02u
#define CASC_T_BUSY        0x80u
#define CASC_T_READY       0x81u
#define CASC_TX_TIMEOUT    50u    /* 1フレーム転送タイムアウト [ms] */
#define CASC_RESP_RETRY    20u    /* 応答ポーリング最大回数 */
#define CASC_RESP_DELAY    2u     /* ポーリング間隔 [ms] */

/* スレーブ用 armed バッファ（送受信は常時1フレームを arm しておく） */
static uint8_t          g_casc_rx[CASC_FRAME_SZ];
static uint8_t          g_casc_tx[CASC_FRAME_SZ];
static volatile uint8_t g_casc_cmd_pending = 0u;
static char             g_casc_cmd_buf[CASC_PAYLOAD_MAX + 1u];

static uint8_t Casc_CRC8(const uint8_t *p, uint32_t len)
{
    uint8_t crc = 0u;
    for (uint32_t i = 0u; i < len; i++) {
        crc ^= p[i];
        for (uint8_t b = 0u; b < 8u; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void Casc_Build(uint8_t *f, uint8_t type, uint8_t box, const uint8_t *payload, uint8_t len)
{
    memset(f, 0, CASC_FRAME_SZ);
    if (len > CASC_PAYLOAD_MAX) len = (uint8_t)CASC_PAYLOAD_MAX;
    f[0] = CASC_SYNC; f[1] = type; f[2] = box; f[3] = len;
    if ((payload != NULL) && (len > 0u)) memcpy(&f[4], payload, len);
    f[CASC_FRAME_SZ - 1u] = Casc_CRC8(f, CASC_FRAME_SZ - 1u);
}

static int Casc_Valid(const uint8_t *f)
{
    return (f[0] == CASC_SYNC) && (Casc_CRC8(f, CASC_FRAME_SZ - 1u) == f[CASC_FRAME_SZ - 1u]);
}

static void Casc_SlaveArm(void)
{
    (void)HAL_SPI_TransmitReceive_IT(&hspi1, g_casc_tx, g_casc_rx, CASC_FRAME_SZ);
}

/* マスター: 他機へコマンド転送し応答を受ける。0=成功, 非0=失敗 */
static int Cascade_Send(uint8_t box, const char *cmd, char *resp, size_t n)
{
    uint8_t txf[CASC_FRAME_SZ], rxf[CASC_FRAME_SZ];

    if (!P_is_master) return -1;

    /* フェーズA: コマンド送信（rx は無視） */
    Casc_Build(txf, CASC_T_CMD, box, (const uint8_t *)cmd, (uint8_t)strlen(cmd));
    if (HAL_SPI_TransmitReceive(&hspi1, txf, rxf, CASC_FRAME_SZ, CASC_TX_TIMEOUT) != HAL_OK) {
        (void)HAL_SPI_Abort(&hspi1);
        return -1;
    }

    /* フェーズB: 応答ポーリング（READY を受けるまで再試行） */
    Casc_Build(txf, CASC_T_POLL, box, NULL, 0u);
    for (uint32_t a = 0u; a < CASC_RESP_RETRY; a++) {
        HAL_Delay(CASC_RESP_DELAY);
        if (HAL_SPI_TransmitReceive(&hspi1, txf, rxf, CASC_FRAME_SZ, CASC_TX_TIMEOUT) != HAL_OK) {
            (void)HAL_SPI_Abort(&hspi1);
            continue;
        }
        if (Casc_Valid(rxf) && (rxf[1] == CASC_T_READY) && (rxf[2] == box)) {
            uint8_t len = rxf[3];
            if (len > CASC_PAYLOAD_MAX) len = (uint8_t)CASC_PAYLOAD_MAX;
            if ((size_t)len >= n) len = (uint8_t)(n - 1u);
            memcpy(resp, &rxf[4], len);
            resp[len] = '\0';
            return 0;
        }
        /* BUSY / 不正フレーム → 再試行 */
    }
    return -1;
}

/* スレーブ: 主ループから呼び、保留コマンドを実行して応答フレームを準備する */
void Cascade_Poll(void)
{
    char     resp[RESP_SZ];
    uint32_t pm;

    if (P_is_master || !g_casc_cmd_pending) return;

    RunCommandLine(g_casc_cmd_buf, resp, sizeof(resp));

    /* armed 転送中の g_casc_tx 書換による分断を最小化（割込禁止で更新, CRC+再試行が保険） */
    pm = __get_PRIMASK();
    __disable_irq();
    Casc_Build(g_casc_tx, CASC_T_READY, P_box_no, (const uint8_t *)resp, (uint8_t)strlen(resp));
    g_casc_cmd_pending = 0u;
    if (!pm) __enable_irq();
}

/* ブート時初期化: 役割に応じ SPI1 を設定（マスター=既定 / スレーブ=Slave 再init＋受信 arm） */
void Cascade_Init(void)
{
    if (P_is_master) return;   /* マスターは CubeMX 既定(Master)のまま, 送信はブロッキング */

    /* スレーブ: SPI1 を Slave/ソフトNSS へ再設定 */
    (void)HAL_SPI_DeInit(&hspi1);
    hspi1.Init.Mode = SPI_MODE_SLAVE;
    hspi1.Init.NSS  = SPI_NSS_SOFT;
    (void)HAL_SPI_Init(&hspi1);

    Casc_Build(g_casc_tx, CASC_T_READY, P_box_no, NULL, 0u);   /* 初期応答=空READY */
    Casc_SlaveArm();
}

/* SPI1 転送完了(スレーブ): 受信フレームを判定し, CMD なら実行待ちに積む */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;
    if (P_is_master) return;

    if (Casc_Valid(g_casc_rx) && (g_casc_rx[2] == P_box_no)) {
        if ((g_casc_rx[1] == CASC_T_CMD) && !g_casc_cmd_pending) {
            uint8_t len = g_casc_rx[3];
            if (len > CASC_PAYLOAD_MAX) len = (uint8_t)CASC_PAYLOAD_MAX;
            memcpy(g_casc_cmd_buf, &g_casc_rx[4], len);
            g_casc_cmd_buf[len] = '\0';
            Casc_Build(g_casc_tx, CASC_T_BUSY, P_box_no, NULL, 0u);  /* 実行前の早すぎるPOLLへBUSY */
            g_casc_cmd_pending = 1u;
        }
        /* POLL は g_casc_tx(=現応答) が既に送出済み。追加処理なし */
    }
    Casc_SlaveArm();   /* 次フレームへ再 arm */
}

/* SPI1 エラー(スレーブ): CS 無しでは OVR 等が起きやすい。中断して再 arm */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;
    if (P_is_master) return;
    (void)HAL_SPI_Abort(hspi);
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    Casc_SlaveArm();
}

/* ── Init / config ────────────────────────────────────────────────────────── */

/*
 * Reg_prepare — parm[] の全エントリをゼロリセットした後、
 * 各パラメータに名前・型・デフォルト値を設定する。
 * フラッシュ読み込み前に呼ぶことで、フラッシュが無効でも
 * Parm_set() がデフォルト値で正常動作できる。
 *
 * 単位: 距離 [μm], 速度 [μm/s], 係数 [μm/pulse]
 */
void Reg_prepare(void)
{
    size_t i;

    /* 全エントリをゼロ初期化 */
    for (i = 0u; i < REG_SIZE; i++) {
        strcpy(parm[i].name,   "");
        parm[i].unit   = UNIT_INT;
        parm[i].data   = 0;
        parm[i].data_f = 0.0f;
        strcpy(parm[i].data_c, "");
    }

    /* ── システム ── */
    strcpy(parm[0].name, "FIRM_VER");  parm[0].unit = UNIT_INT;   parm[0].data   = 1;
    strcpy(parm[1].name, "BOX_NO");    parm[1].unit = UNIT_INT;   parm[1].data   = 0;
    /* カスケード接続: 役割・台数（GUI/コマンドで設定→RP/RS） */
    strcpy(parm[2].name, "IS_MASTER"); parm[2].unit = UNIT_INT;   parm[2].data   = 1;   /* 1=マスター(既定) */
    strcpy(parm[3].name, "NUM_BOXES"); parm[3].unit = UNIT_INT;   parm[3].data   = 2;   /* 接続台数(初期2) */

    /* ── モーター全体 ── */
    strcpy(parm[39].name, "INIT_TOUT");  parm[39].unit = UNIT_INT;   parm[39].data   = 100;      /* ×10 ms = 1000 ms */

    /* ── ホームオフセット [μm] ── */
    strcpy(parm[56].name, "HOME_OFS_X"); parm[56].unit = UNIT_FLOAT; parm[56].data_f = 0.0f;
    strcpy(parm[57].name, "HOME_OFS_Y"); parm[57].unit = UNIT_FLOAT; parm[57].data_f = 0.0f;
    strcpy(parm[58].name, "HOME_OFS_Z"); parm[58].unit = UNIT_FLOAT; parm[58].data_f = 0.0f;

    /* ── CW ソフトリミット [μm] ── */
    strcpy(parm[72].name, "LIM_CW_X");  parm[72].unit = UNIT_FLOAT; parm[72].data_f = 25000.0f;
    strcpy(parm[73].name, "LIM_CW_Y");  parm[73].unit = UNIT_FLOAT; parm[73].data_f = 25000.0f;
    strcpy(parm[74].name, "LIM_CW_Z");  parm[74].unit = UNIT_FLOAT; parm[74].data_f = 25000.0f;

    /* ── CCW ソフトリミット [μm] ── */
    strcpy(parm[88].name, "LIM_CCW_X"); parm[88].unit = UNIT_FLOAT; parm[88].data_f = 0.0f;
    strcpy(parm[89].name, "LIM_CCW_Y"); parm[89].unit = UNIT_FLOAT; parm[89].data_f = 0.0f;
    strcpy(parm[90].name, "LIM_CCW_Z"); parm[90].unit = UNIT_FLOAT; parm[90].data_f = 0.0f;

    /* ── 換算係数 分子 [μm]:  μm/pulse = coeff_num / coeff_den ── */
    strcpy(parm[104].name, "COEF_NUM_X"); parm[104].unit = UNIT_FLOAT; parm[104].data_f = 1.0f;
    strcpy(parm[105].name, "COEF_NUM_Y"); parm[105].unit = UNIT_FLOAT; parm[105].data_f = 1.0f;
    strcpy(parm[106].name, "COEF_NUM_Z"); parm[106].unit = UNIT_FLOAT; parm[106].data_f = 1.0f;

    /* ── 換算係数 分母 [pulse] ── */
    strcpy(parm[120].name, "COEF_DEN_X"); parm[120].unit = UNIT_FLOAT; parm[120].data_f = 1.0f;
    strcpy(parm[121].name, "COEF_DEN_Y"); parm[121].unit = UNIT_FLOAT; parm[121].data_f = 1.0f;
    strcpy(parm[122].name, "COEF_DEN_Z"); parm[122].unit = UNIT_FLOAT; parm[122].data_f = 1.0f;

    /* ── ビットマスク ── */
    strcpy(parm[183].name, "INIT_ACC");  parm[183].unit = UNIT_INT; parm[183].data = 0;   /* 自動ホーミングなし */
    strcpy(parm[184].name, "MOTOR_EN");  parm[184].unit = UNIT_INT; parm[184].data = 7;   /* 0b111: 全軸有効 */

    /* ── 初速度 [pps] ── */
    strcpy(parm[136].name, "START_PPS_X"); parm[136].unit = UNIT_INT; parm[136].data = 100;
    strcpy(parm[137].name, "START_PPS_Y"); parm[137].unit = UNIT_INT; parm[137].data = 100;
    strcpy(parm[138].name, "START_PPS_Z"); parm[138].unit = UNIT_INT; parm[138].data = 100;

    /* ── 加速時間 [ms] ── */
    strcpy(parm[152].name, "ACCEL_MS_X"); parm[152].unit = UNIT_INT; parm[152].data = 100;
    strcpy(parm[153].name, "ACCEL_MS_Y"); parm[153].unit = UNIT_INT; parm[153].data = 100;
    strcpy(parm[154].name, "ACCEL_MS_Z"); parm[154].unit = UNIT_INT; parm[154].data = 100;

    /* ── 分解能 (CVD RESOLUTION レジスタ値 = microstep×10, 既定 microstep100) ── */
    strcpy(parm[168].name, "RESOL_X"); parm[168].unit = UNIT_INT; parm[168].data = 1000;
    strcpy(parm[169].name, "RESOL_Y"); parm[169].unit = UNIT_INT; parm[169].data = 1000;
    strcpy(parm[170].name, "RESOL_Z"); parm[170].unit = UNIT_INT; parm[170].data = 1000;

    /* ── モーター型番 (CVD MOT_SEL, 既定 0.75A=0xFE01) ── */
    strcpy(parm[200].name, "MOTSEL_X"); parm[200].unit = UNIT_INT; parm[200].data = 0xFE01;
    strcpy(parm[201].name, "MOTSEL_Y"); parm[201].unit = UNIT_INT; parm[201].data = 0xFE01;
    strcpy(parm[202].name, "MOTSEL_Z"); parm[202].unit = UNIT_INT; parm[202].data = 0xFE01;
}

void GetCLK(void)
{
    RCC_ClkInitTypeDef clk_init;
    uint32_t flash_latency;

    sysclk_hz   = HAL_RCC_GetSysClockFreq();
    apb1_clocks = HAL_RCC_GetPCLK1Freq();
    apb2_clocks = HAL_RCC_GetPCLK2Freq();
    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);

    apb1_timer_clocks = (clk_init.APB1CLKDivider == RCC_HCLK_DIV1)
                        ? apb1_clocks : apb1_clocks * 2u;
    apb2_timer_clocks = (clk_init.APB2CLKDivider == RCC_HCLK_DIV1)
                        ? apb2_clocks : apb2_clocks * 2u;

    tim1_psc  = htim1.Instance->PSC + 1u;
    tim8_psc  = htim8.Instance->PSC + 1u;
    tim6_psc  = htim6.Instance->PSC + 1u;
    tim15_psc = htim15.Instance->PSC + 1u;

    tim1_clk  = apb2_timer_clocks / tim1_psc;
    tim8_clk  = apb2_timer_clocks / tim8_psc;    /* TIM8 は APB2 */
    tim6_clk  = apb1_timer_clocks / tim6_psc;
    tim15_clk = apb2_timer_clocks / tim15_psc;   /* TIM15 は APB2 */
}

void Motor_Init(void)
{
    uint8_t i;
    for (i = 0u; i < MOTOR_MAX; i++) {
        memset(&g_motor[i], 0, sizeof(MotorCtrl_t));
        g_motor[i].state = MOTOR_IDLE;
        g_homed[i]      = 0u;   /* 起動直後は未ホーミング＝リミット非適用 */
        P_limit_cw[i]   = INT32_MAX;
        P_limit_ccw[i]  = INT32_MIN;
    }
}

void Motor_Config(void)
{
    uint8_t i;
    for (i = 0u; i < MOTOR_MAX; i++) CVD_InitPins(g_cvd[i]);
    for (i = 0u; i < MOTOR_MAX; i++) {
        if (P_motor_en[i])
            (void)CVD_ApplyConfigAndEnable(g_cvd[i], &g_cvd_cfg[i]);
    }
}

/* ── Parameter register / application parameters ─────────────────────────── */

/* 4-byte aligned so DataAddress for HAL_FLASH_Program is valid */
reg_t    parm[REG_SIZE] __attribute__((aligned(4)));
uint8_t  P_firm_no, P_box_no;
uint8_t  P_is_master = 1u, P_num_boxes = 2u;   /* カスケード役割・台数 */
uint32_t P_init_timeout;
float    P_motor_coeff[MOTOR_MAX];
int32_t  P_home_offset[MOTOR_MAX];
int32_t  P_limit_cw[MOTOR_MAX], P_limit_ccw[MOTOR_MAX];
uint8_t  P_init_access[MOTOR_MAX], P_motor_en[MOTOR_MAX];
int32_t  P_start_pps[MOTOR_MAX], P_accel_ms[MOTOR_MAX];

void Parm_set(void)
{
    uint8_t  i;
    uint16_t tmp16 = 0x0001u;

    P_firm_no      = (uint8_t)parm[0].data;
    P_box_no       = (uint8_t)parm[1].data;
    P_is_master    = (parm[2].data != 0) ? 1u : 0u;
    P_num_boxes    = (parm[3].data > 0) ? (uint8_t)parm[3].data : 1u;
    P_init_timeout = (uint32_t)(parm[39].data / 10);

    for (i = 0u; i < MOTOR_MAX; i++) {
        P_motor_coeff[i] = (parm[120 + i].data_f != 0.0f)
                           ? parm[104 + i].data_f / parm[120 + i].data_f
                           : 1.0f;
        if (P_motor_coeff[i] != 0.0f) {
            P_home_offset[i] = (int32_t)(parm[56 + i].data_f / P_motor_coeff[i]);
            P_limit_cw[i]    = (int32_t)(parm[72 + i].data_f / P_motor_coeff[i]);
            P_limit_ccw[i]   = (int32_t)(parm[88 + i].data_f / P_motor_coeff[i]);
        }
        P_init_access[i] = (((uint16_t)parm[183].data & tmp16) != 0u) ? 1u : 0u;
        P_motor_en[i]    = (((uint16_t)parm[184].data & tmp16) != 0u) ? 1u : 0u;
        tmp16 <<= 1;

        /* モーション軸別パラメータ */
        P_start_pps[i] = parm[136 + i].data;
        P_accel_ms[i]  = parm[152 + i].data;

        /* 軸別 CVD 設定: ベースプロファイル + 分解能・型番を上書き */
        g_cvd_cfg[i]            = CVD_Profile_PG413M_LA_C;
        g_cvd_cfg[i].resolution = (uint16_t)parm[168 + i].data;
        g_cvd_cfg[i].mot_sel    = (uint16_t)parm[200 + i].data;
    }

    /* P_motor_en / g_cvd_cfg 確定後に CVD ドライバを設定・有効化 */
    Motor_Config();
}

/* ── Flash parameter storage ──────────────────────────────────────────────── */

/*
 * Flash layout at REG_ADDR (0x08100000), Bank 2 Sector 0 (8 KB):
 *   [0x00-0x0F]  ParmFlashHeader_t  (16 bytes = 1 flash word)
 *   [0x10-0x1C0F] parm[REG_SIZE]   (7168 bytes = 448 flash words)
 *   Total: 7184 bytes < 8192 bytes (sector size)
 *
 * Flash word = 128 bits = 16 bytes (FLASH_NB_32BITWORD_IN_FLASHWORD = 4)
 * HAL_FLASH_Program writes 16 bytes; DataAddress must be 4-byte aligned.
 */

typedef struct {
    uint32_t magic;     /* REG_FLASH_MAGIC */
    uint32_t size;      /* sizeof(parm)    */
    uint32_t crc32;     /* CRC32 of parm   */
    uint32_t reserved;
} ParmFlashHeader_t;

/* CRC32 (polynomial 0x04C11DB7, reflected-input variant) */
static uint32_t CalcCRC32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    for (i = 0u; i < len; i++) {
        uint8_t b = p[i];
        uint8_t bit;
        for (bit = 0u; bit < 8u; bit++) {
            if (((crc >> 31) ^ (b >> 7)) & 1u) {
                crc = (crc << 1) ^ 0x04C11DB7u;
            } else {
                crc <<= 1;
            }
            b <<= 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/*
 * Load parm[] from flash.
 * Returns HAL_OK if valid data was found and copied, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef Parm_Flash_Load(void)
{
    const ParmFlashHeader_t *hdr = (const ParmFlashHeader_t *)REG_ADDR;

    if (hdr->magic != REG_FLASH_MAGIC)  return HAL_ERROR;
    if (hdr->size  != sizeof(parm))     return HAL_ERROR;

    const void *data_ptr = (const void *)(REG_ADDR + sizeof(ParmFlashHeader_t));
    if (CalcCRC32(data_ptr, sizeof(parm)) != hdr->crc32) return HAL_ERROR;

    memcpy(parm, data_ptr, sizeof(parm));
    return HAL_OK;
}

/*
 * Save parm[] to flash.
 * Erases Bank 2 Sector 0, writes header + parm[] in 16-byte flash words.
 */
HAL_StatusTypeDef Parm_Flash_Save(void)
{
    /* Build 16-byte header */
    ParmFlashHeader_t hdr __attribute__((aligned(4))) = {
        .magic    = REG_FLASH_MAGIC,
        .size     = (uint32_t)sizeof(parm),
        .crc32    = CalcCRC32(parm, sizeof(parm)),
        .reserved = 0u,
    };

    if (HAL_FLASH_Unlock() != HAL_OK) return HAL_ERROR;

    /* Erase Bank 2 Sector 0 (8 KB at 0x08100000)
     * 注: STM32H7A3 は 128bit(16B) 固定書き込みで PSIZE を持たないため
     *     VoltageRange (FLASH_VOLTAGE_RANGE_*) は存在せず未使用。0 のままでよい。 */
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Banks        = REG_BANK,
        .Sector       = REG_SECTOR,
        .NbSectors    = 1u,
    };
    uint32_t sector_error = 0u;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    /* Write header (1 flash word = 16 bytes) */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          REG_ADDR,
                          (uint32_t)&hdr) != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    /* Write parm[] — 7168 bytes / 16 = 448 flash words */
    uint32_t flash_addr = REG_ADDR + sizeof(ParmFlashHeader_t);
    uint32_t data_addr  = (uint32_t)parm;
    uint32_t words      = sizeof(parm) / 16u;   /* exact: 7168/16 = 448 */
    uint32_t w;
    for (w = 0u; w < words; w++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              flash_addr,
                              data_addr) != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return HAL_ERROR;
        }
        flash_addr += 16u;
        data_addr  += 16u;
    }

    (void)HAL_FLASH_Lock();
    return HAL_OK;
}
