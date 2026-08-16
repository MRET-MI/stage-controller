/*
 * functions.h
 *
 *  Created on: 2026/05/28
 *      Author: tm472
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include <stdint.h>
#include <stddef.h>
#include "cvd_driver.h"

/* Unit type */
#define UNIT_INT    0
#define UNIT_FLOAT  1
#define UNIT_CHAR   2

/* Register */
#define REG_NAME_SIZE  12
#define REG_SIZE       256

/*
 * Parameter flash storage — Bank 2, Sector 0 (0x08100000, 8 KB)
 * Flash word = 128 bit = 16 bytes (FLASH_NB_32BITWORD_IN_FLASHWORD = 4)
 * Layout: 16-byte header + parm[256] (7168 bytes) = 7184 bytes < 8192 bytes
 */
#define REG_BANK       FLASH_BANK_2   /* Bank 2 starts at 0x08100000          */
#define REG_SECTOR     0u             /* Sector 0 of Bank 2                    */
#define REG_ADDR       0x08100000u    /* = FLASH_BASE + FLASH_BANK_SIZE        */
#define REG_FLASH_MAGIC 0x50524D41u  /* "PRMA" — validity marker              */

/* Buffer size */
#define BUFF_RX_SIZE   4096
#define TEX_BUFF       20480
#define MOTOR_MAX      3

/* DMA buffer: 1 word (ARR) per pulse — used for accel/decel ramps only.
 * Accel/decel longer than this are split into chained blocks. */
//#define MOTOR_DMA_BUF_SIZE   8192u
#define MOTOR_DMA_BUF_SIZE   16386u

/* Timer counter clock [Hz] fixed by prescaler */
#define MOTOR_TIM_CLK_HZ     1000000UL

/* Minimum speed [pps] for profile calculations */
#define MOTOR_SPEED_MIN_F    1.0f

/* Maximum pulse rate [pps] safety clamp（誤設定で 500kHz 等を要求されても抑止） */
#define MOTOR_MAX_PPS        200000u

/* RCR is 16-bit: at most 65536 pulses per update event */
#define MOTOR_RCR_MAX        65536u

/* ── センサ論理 ──────────────────────────────────────────────────────────────
 * 駿河精機のセンサは Normally Closed(NC):
 *   非検出=導通(Low) / 検出・断線=開放→プルアップで High。
 * よって「検出=High」。内部プルアップ(CubeMX)はNCのオープンコレクタに適合。
 *   1: 検出=High (NC, 駿河精機)
 *   0: 検出=Low  (NO, 負論理) */
#define SENSOR_ACTIVE_HIGH   1

/* ── Homing（CCW limit 基準）─────────────────────────────────────────────────
 * 各サブ移動は一定速(cruise-only, DMA非動作)で行い EXTI で即停止する。
 * CCW limit センサを原点基準にする: CCW(負)へ高速接近→検出で停止→CW(正)へ離脱
 * （センサ解除）→CCWへ低速再接近→検出で pos=0。CW limit は過走防止(異常)。 */
#define HOME_FAST_PPS        10000u  /* 高速探索速度（無加速で起動可能な範囲）*/
#define HOME_SLOW_PPS        300u    /* 低速再探索・バックオフ速度            */
#define HOME_SEEK_DIR        0u      /* CCW limit 探索方向 0=CCW(負)          */
#define HOME_MAX_TRAVEL      4000000 /* 探索の最大移動量[pulse]（保険・未検出検出）*/

typedef uint8_t unit_t;

typedef struct st_reg
{
    char    name[REG_NAME_SIZE];
    unit_t  unit;
    int32_t data;
    char    data_c[4];
    float   data_f;
} reg_t;

/* ── Motor control ─────────────────────────────────────────────────────────── */

typedef enum {
    MOTOR_IDLE    = 0,
    MOTOR_MOVING  = 1,
    MOTOR_HOMING  = 2,
    MOTOR_JOGGING = 3,
} MotorState_t;

/* 駆動フェーズ: 加速/減速=毎パルスDMA(RCR=0), 巡航=RCRカウント(DMAなし) */
typedef enum {
    PH_IDLE   = 0,
    PH_ACCEL  = 1,
    PH_CRUISE = 2,
    PH_DECEL  = 3,
} MotorPhase_t;

/* ホーミング状態（軸ごと, CCW limit 基準） */
typedef enum {
    HM_IDLE      = 0,
    HM_FAST_SEEK = 1,   /* CCWへ高速接近（CCW limit作動で停止） */
    HM_BACKOFF   = 2,   /* CWへ離脱（CCW limit解除で停止） */
    HM_SLOW_SEEK = 3,   /* CCWへ低速接近（CCW limit作動で停止→原点）*/
    HM_DONE      = 4,
    HM_ERROR     = 5,
} HomingState_t;

/* EXTI から MotorStateTick へ渡すイベント種別 */
typedef enum {
    HEVT_NONE   = 0,
    HEVT_HOME   = 1,   /* CCW limit エッジで停止した（原点基準センサ） */
    HEVT_LIMIT  = 2,   /* CW 端で停止した（過走防止・異常）*/
} HomeEvt_t;

typedef struct {
    /* Axis state & position */
    volatile MotorState_t   state;
    volatile int32_t        pos;          /* current position [pulses] */
    int32_t                 start_pos;    /* pos at move start          */
    int32_t                 target;       /* target position [pulses]  */
    uint8_t                 dir;          /* 1=CW, 0=CCW               */

    /* Motion profile (computed at move start) */
    uint32_t                total_pulses;
    float                   peak_speed;   /* [pps]                     */
    float                   start_pps;    /* start/end speed [pps]      */
    float                   accel_pps2;   /* acceleration [pps^2]       */
    float                   accel_pulses; /* accel distance (float, for GetSpeedAtPulse) */
    float                   decel_pulses; /* decel distance (float)     */

    /* Integer phase pulse counts */
    uint32_t                accel_n;      /* accel pulses */
    uint32_t                cruise_n;     /* cruise pulses */
    uint32_t                decel_n;      /* decel pulses */

    /* Runtime phase state */
    volatile MotorPhase_t   phase;
    uint32_t                seg_issued;   /* pulses issued in current ramp phase (accel/decel) */
    volatile uint32_t       cruise_remain;/* cruise pulses left to count via RCR */
    uint32_t                block_pulses; /* pulses in the in-flight DMA block */
    volatile uint8_t        dma_running;
} MotorCtrl_t;

/* ── Function prototypes ───────────────────────────────────────────────────── */

void GetCLK(void);
void Reg_prepare(void);
void Parm_set(void);
void Motor_Config(void);
void Motor_Init(void);

HAL_StatusTypeDef Parm_Flash_Load(void);
HAL_StatusTypeDef Parm_Flash_Save(void);

/* Phase-engine callback — dispatched from HAL_TIM_PeriodElapsedCallback
 * (fires on DMA block完了[accel/decel] と 巡航UEV[cruise]) */
void Motor_OnUpdateEvent(TIM_HandleTypeDef *htim);

/* 10 ms state tick */
void MotorStateTick(void);

/* Command processing */
void Cmd_Process(void);

/* 1行のコマンド（; 区切り可, @<box> は含めない）を実行し応答文字列を生成 */
void RunCommandLine(const char *line, char *resp_out, size_t n);

/* カスケード接続（コントローラ間 SPI, hspi1）
 *   Cascade_Init: ブート時に役割へ応じ SPI1 を設定（main.c から MX_SPI1_Init 後に呼ぶ）
 *   Cascade_Poll: スレーブ機の主ループから呼び, 受信コマンドを実行して応答を用意 */
void Cascade_Init(void);
void Cascade_Poll(void);

/* ── Exported variables ────────────────────────────────────────────────────── */

extern reg_t parm[REG_SIZE];

extern uint32_t sysclk_hz;
extern uint32_t apb1_clocks, apb2_clocks;
extern uint32_t apb1_timer_clocks, apb2_timer_clocks;
extern uint32_t tim1_psc, tim3_psc, tim6_psc, tim8_psc;
extern uint32_t tim1_clk, tim3_clk, tim6_clk, tim8_clk;

extern CVD_HandleTypeDef  g_axis1_cvd, g_axis2_cvd, g_axis3_cvd;
extern CVD_HandleTypeDef *g_cvd[MOTOR_MAX];

extern MotorCtrl_t g_motor[MOTOR_MAX];
extern volatile uint32_t g_speed_pps;
extern volatile float    g_speed_mm;
extern volatile uint8_t  g_usb_rx_ready;

extern volatile HomingState_t g_homing[MOTOR_MAX];

extern uint8_t  P_firm_no, P_box_no;
extern uint8_t  P_is_master, P_num_boxes;   /* カスケード役割・台数 */
extern uint32_t P_init_timeout;
extern float    P_motor_coeff[MOTOR_MAX];
extern int32_t  P_home_offset[MOTOR_MAX];
extern int32_t  P_limit_cw[MOTOR_MAX], P_limit_ccw[MOTOR_MAX];
extern uint8_t  P_init_access[MOTOR_MAX], P_motor_en[MOTOR_MAX];
extern int32_t  P_start_pps[MOTOR_MAX], P_accel_ms[MOTOR_MAX];

/* USB CDC helpers (defined in usbd_cdc_if.c) */
uint8_t CDC_RxReadByte(uint8_t *b);
void    CDC_SendString(const char *str);

#endif /* INC_FUNCTIONS_H_ */
