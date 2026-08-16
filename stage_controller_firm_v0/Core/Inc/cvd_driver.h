/**
 * @file cvd_driver.h
 * @brief Oriental Motor CVD S-type SPI setting driver for STM32 HAL.
 *
 * Target:
 *   - Driver: CVD S-type, pulse train input, SPI setting
 *   - Drive : PLS/DIR or CW/CCW pulse input
 *   - ENABLE: pin input or SPI NET-IN bit
 *
 * CubeMX SPI setting:
 *   - Master
 *   - Motorola frame format
 *   - Data Size: 8 bits
 *   - First Bit: MSB first
 *   - CPOL: High
 *   - CPHA: 2 Edge
 *   - NSS: Software
 *   - Baud rate: <= 1 MHz
 */

#ifndef CVD_DRIVER_H
#define CVD_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define CVD_CMD_NOP          (0x00u)
#define CVD_CMD_WRITE        (0x02u)
#define CVD_CMD_READ         (0x03u)
#define CVD_CMD_ACTIVATE     (0x08u)
#define CVD_CMD_DEACTIVATE   (0x09u)

#define CVD_REG_NET_IN       (0x02u)
#define CVD_REG_RUN_CRNT     (0x04u)
#define CVD_REG_STOP_CRNT    (0x06u)
#define CVD_REG_SETTING      (0x0Au)
#define CVD_REG_RESOLUTION   (0x0Cu)
#define CVD_REG_MOT_SEL      (0x0Eu)

#define CVD_REG_ALM_CD       (0x20u)
#define CVD_REG_NET_OUT      (0x22u)

#define CVD_NET_IN_FIL_EN        (1u << 9)
#define CVD_NET_IN_SD_EN         (1u << 8)
#define CVD_NET_IN_ERR_CLR       (1u << 3)
#define CVD_NET_IN_ALM_RST       (1u << 2)
#define CVD_NET_IN_ENABLE        (1u << 1)

#define CVD_NET_IN_SD_ON_FILTER_OFF   (CVD_NET_IN_SD_EN)
#define CVD_NET_IN_SD_ON_FILTER_ON    (CVD_NET_IN_SD_EN | CVD_NET_IN_FIL_EN)
#define CVD_NET_IN_VERIFY_MASK        (CVD_NET_IN_FIL_EN | CVD_NET_IN_SD_EN | CVD_NET_IN_ENABLE)

#define CVD_SETTING_PULSE_SRC_SPI         (1u << 14)
#define CVD_SETTING_SD_SRC_SPI            (1u << 13)
#define CVD_SETTING_RUN_CRNT_SRC_SPI      (1u << 12)
#define CVD_SETTING_STOP_CRNT_SRC_SPI     (1u << 11)
#define CVD_SETTING_MOT_SEL_SRC_SPI       (1u << 10)
#define CVD_SETTING_ENABLE_SRC_IO         (0u << 9)
#define CVD_SETTING_ENABLE_SRC_SPI        (1u << 9)
#define CVD_SETTING_PULSE_1P_PLS_DIR      (0u << 6)
#define CVD_SETTING_PULSE_2P_CW_CCW       (1u << 6)
#define CVD_SETTING_STOP_CRNT_BASE_RATED  (0u << 3)
#define CVD_SETTING_STOP_CRNT_BASE_RUN    (1u << 3)

/* B14,B13,B12,B11,B10,B9,B6,B3 */
#define CVD_SETTING_VERIFY_MASK           ((uint16_t)0x7E48u)

#define CVD_SETTING_ENABLE_PIN_PLS_DIR        ( \
    CVD_SETTING_PULSE_SRC_SPI                 | \
    CVD_SETTING_SD_SRC_SPI                    | \
    CVD_SETTING_RUN_CRNT_SRC_SPI              | \
    CVD_SETTING_STOP_CRNT_SRC_SPI             | \
    CVD_SETTING_MOT_SEL_SRC_SPI               | \
    CVD_SETTING_ENABLE_SRC_IO                 | \
    CVD_SETTING_PULSE_1P_PLS_DIR              | \
    CVD_SETTING_STOP_CRNT_BASE_RUN              \
)

#define CVD_SETTING_ENABLE_SPI_PLS_DIR        ( \
    CVD_SETTING_ENABLE_PIN_PLS_DIR             | \
    CVD_SETTING_ENABLE_SRC_SPI                   \
)

#define CVD_STATUS0_STATE_REQ_MASK       (0xC000u)
#define CVD_STATUS0_STATE_REQ_SETTING    (0x4000u)
#define CVD_STATUS0_STATE_REQ_OPERATION  (0x8000u)
#define CVD_STATUS0_MF_ERR               (1u << 13)
#define CVD_STATUS0_HDR_ERR              (1u << 12)
#define CVD_STATUS0_PRM_ERR              (1u << 10)
#define CVD_STATUS0_CMD_ERR              (1u << 9)
#define CVD_STATUS0_EXE_ERR              (1u << 8)
#define CVD_STATUS0_ERR_CNT_MASK         (0x00FFu)

#define CVD_STATUS1_STATE_OPERATION      (1u << 15)
#define CVD_STATUS1_ALM_CD_MASK          (0x00FFu)

#define CVD_NET_OUT_PWR_ON               (1u << 5)
#define CVD_NET_OUT_CRNT_ON              (1u << 3)
#define CVD_NET_OUT_PLS_RDY              (1u << 2)
#define CVD_NET_OUT_ALM                  (1u << 0)

#define CVD_CURRENT_PERCENT_X10(x10)     ((uint16_t)(x10))
#define CVD_CURRENT_PERCENT(percent)     ((uint16_t)((percent) * 10u))
#define CVD_RUN_CURRENT_100_PERCENT      ((uint16_t)1000u)
#define CVD_RUN_CURRENT_80_PERCENT       ((uint16_t)800u)
#define CVD_RUN_CURRENT_50_PERCENT       ((uint16_t)500u)
#define CVD_STOP_CURRENT_50_PERCENT      ((uint16_t)500u)
#define CVD_STOP_CURRENT_25_PERCENT      ((uint16_t)250u)

#define CVD_RESOLUTION_FROM_PPR(ppr)     ((uint16_t)((ppr) / 50u))
#define CVD_5PH_MICROSTEP_1              CVD_RESOLUTION_FROM_PPR(500u)
#define CVD_5PH_MICROSTEP_2              CVD_RESOLUTION_FROM_PPR(1000u)
#define CVD_5PH_MICROSTEP_4              CVD_RESOLUTION_FROM_PPR(2000u)
#define CVD_5PH_MICROSTEP_5              CVD_RESOLUTION_FROM_PPR(2500u)
#define CVD_5PH_MICROSTEP_10             CVD_RESOLUTION_FROM_PPR(5000u)
#define CVD_5PH_MICROSTEP_20             CVD_RESOLUTION_FROM_PPR(10000u)
#define CVD_5PH_MICROSTEP_50             CVD_RESOLUTION_FROM_PPR(25000u)
#define CVD_5PH_MICROSTEP_100            CVD_RESOLUTION_FROM_PPR(50000u)

#define CVD5_MOT_SEL_0P35A               ((uint16_t)0xFF00u)
#define CVD5_MOT_SEL_0P75A               ((uint16_t)0xFE01u)
#define CVD5_MOT_SEL_1P20A               ((uint16_t)0xFD02u)
#define CVD5_MOT_SEL_1P40A               ((uint16_t)0xFC03u)
#define CVD5_MOT_SEL_1P80A               ((uint16_t)0xFB04u)
#define CVD5_MOT_SEL_2P40A               ((uint16_t)0xFA05u)

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
} CVD_HandleTypeDef;

typedef struct {
    uint16_t mot_sel;
    uint16_t resolution;
    uint16_t run_current;
    uint16_t stop_current;
    uint16_t setting;
    uint16_t net_in;
} CVD_ConfigTypeDef;

void CVD_InitPins(CVD_HandleTypeDef *cvd);
void CVD_SetDirCW(CVD_HandleTypeDef *cvd);
void CVD_SetDirCCW(CVD_HandleTypeDef *cvd);
HAL_StatusTypeDef CVD_Nop(CVD_HandleTypeDef *cvd, uint16_t *status0, uint16_t *status1);
HAL_StatusTypeDef CVD_WriteReg16(CVD_HandleTypeDef *cvd, uint8_t addr, uint16_t data);
HAL_StatusTypeDef CVD_ReadReg16(CVD_HandleTypeDef *cvd, uint8_t addr, uint16_t *data);
HAL_StatusTypeDef CVD_Activate(CVD_HandleTypeDef *cvd);
HAL_StatusTypeDef CVD_Deactivate(CVD_HandleTypeDef *cvd);
HAL_StatusTypeDef CVD_ClearCommError(CVD_HandleTypeDef *cvd, uint16_t net_in_base);
HAL_StatusTypeDef CVD_ClearAlarm(CVD_HandleTypeDef *cvd, uint16_t net_in_base);
HAL_StatusTypeDef CVD_EnableMotorPin(CVD_HandleTypeDef *cvd);
HAL_StatusTypeDef CVD_DisableMotorPin(CVD_HandleTypeDef *cvd);
HAL_StatusTypeDef CVD_VerifyConfig(CVD_HandleTypeDef *cvd, const CVD_ConfigTypeDef *cfg);
HAL_StatusTypeDef CVD_ApplyConfig(CVD_HandleTypeDef *cvd, const CVD_ConfigTypeDef *cfg);
HAL_StatusTypeDef CVD_ApplyConfigAndEnable(CVD_HandleTypeDef *cvd, const CVD_ConfigTypeDef *cfg);
HAL_StatusTypeDef CVD_ReadNetOut(CVD_HandleTypeDef *cvd, uint16_t *net_out);
HAL_StatusTypeDef CVD_ReadAlarmCode(CVD_HandleTypeDef *cvd, uint16_t *alarm_code);
HAL_StatusTypeDef CVD_WaitOperationState(CVD_HandleTypeDef *cvd, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* CVD_DRIVER_H */
