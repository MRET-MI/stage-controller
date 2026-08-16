/**
 * @file cvd_driver.c
 * @brief Oriental Motor CVD S-type SPI setting driver for STM32 HAL.
 */

#include "../Inc/cvd_driver.h"
#include <string.h>

static void CVD_CS_Low(CVD_HandleTypeDef *cvd)
{
    if ((cvd != NULL) && (cvd->cs_port != NULL)) {
        HAL_GPIO_WritePin(cvd->cs_port, cvd->cs_pin, GPIO_PIN_RESET);
    }
}

static void CVD_CS_High(CVD_HandleTypeDef *cvd)
{
    if ((cvd != NULL) && (cvd->cs_port != NULL)) {
        HAL_GPIO_WritePin(cvd->cs_port, cvd->cs_pin, GPIO_PIN_SET);
    }
}

static HAL_StatusTypeDef CVD_SPI_TransmitReceive(
    CVD_HandleTypeDef *cvd,
    const uint8_t *tx,
    uint8_t *rx,
    uint16_t len
)
{
    HAL_StatusTypeDef ret;

    if ((cvd == NULL) || (cvd->hspi == NULL) || (cvd->cs_port == NULL) ||
        (tx == NULL) || (rx == NULL) || (len == 0u)) {
        return HAL_ERROR;
    }

    CVD_CS_Low(cvd);

    for (volatile uint32_t i = 0; i < 200u; i++) {
        __NOP();
    }

    ret = HAL_SPI_TransmitReceive(cvd->hspi, (uint8_t *)tx, rx, len, 100u);

    for (volatile uint32_t i = 0; i < 500u; i++) {
        __NOP();
    }

    CVD_CS_High(cvd);

    HAL_Delay(2u);

    return ret;
}

void CVD_InitPins(CVD_HandleTypeDef *cvd)
{
    if (cvd == NULL) {
        return;
    }
    if (cvd->cs_port != NULL) {
        CVD_CS_High(cvd);
    }
    (void)CVD_DisableMotorPin(cvd);
    CVD_SetDirCW(cvd);
}

void CVD_SetDirCW(CVD_HandleTypeDef *cvd)
{
    if ((cvd != NULL) && (cvd->dir_port != NULL)) {
        HAL_GPIO_WritePin(cvd->dir_port, cvd->dir_pin, GPIO_PIN_SET);
    }
}

void CVD_SetDirCCW(CVD_HandleTypeDef *cvd)
{
    if ((cvd != NULL) && (cvd->dir_port != NULL)) {
        HAL_GPIO_WritePin(cvd->dir_port, cvd->dir_pin, GPIO_PIN_RESET);
    }
}

HAL_StatusTypeDef CVD_Nop(CVD_HandleTypeDef *cvd, uint16_t *status0, uint16_t *status1)
{
    uint8_t tx[4] = { CVD_CMD_NOP, 0x00u, 0x00u, 0x00u };
    uint8_t rx[4] = {0};
    HAL_StatusTypeDef ret = CVD_SPI_TransmitReceive(cvd, tx, rx, sizeof(tx));
    if (ret != HAL_OK) return ret;
    if (status0 != NULL) *status0 = ((uint16_t)rx[0] << 8) | rx[1];
    if (status1 != NULL) *status1 = ((uint16_t)rx[2] << 8) | rx[3];
    return HAL_OK;
}

HAL_StatusTypeDef CVD_WriteReg16(CVD_HandleTypeDef *cvd, uint8_t addr, uint16_t data)
{
    uint8_t tx[6];
    uint8_t rx[6];
    tx[0] = CVD_CMD_WRITE;
    tx[1] = (uint8_t)(addr & 0xFEu);
    tx[2] = 0x00u;
    tx[3] = 0x00u;
    tx[4] = (uint8_t)(data >> 8);
    tx[5] = (uint8_t)(data & 0xFFu);
    memset(rx, 0, sizeof(rx));
    return CVD_SPI_TransmitReceive(cvd, tx, rx, sizeof(tx));
}

HAL_StatusTypeDef CVD_ReadReg16(CVD_HandleTypeDef *cvd, uint8_t addr, uint16_t *data)
{
    uint8_t tx[6];
    uint8_t rx[6];
    if (data == NULL) return HAL_ERROR;
    tx[0] = CVD_CMD_READ;
    tx[1] = (uint8_t)(addr & 0xFEu);
    tx[2] = 0x00u;
    tx[3] = 0x00u;
    tx[4] = 0x00u;
    tx[5] = 0x00u;
    memset(rx, 0, sizeof(rx));
    HAL_StatusTypeDef ret = CVD_SPI_TransmitReceive(cvd, tx, rx, sizeof(tx));
    if (ret != HAL_OK) return ret;
    *data = ((uint16_t)rx[4] << 8) | rx[5];
    return HAL_OK;
}

static HAL_StatusTypeDef CVD_SendCommand(CVD_HandleTypeDef *cvd, uint8_t cmd)
{
    uint8_t tx[4] = { cmd, 0x00u, 0x00u, 0x00u };
    uint8_t rx[4] = {0};
    return CVD_SPI_TransmitReceive(cvd, tx, rx, sizeof(tx));
}

HAL_StatusTypeDef CVD_Activate(CVD_HandleTypeDef *cvd)
{
    HAL_StatusTypeDef ret = CVD_SendCommand(cvd, CVD_CMD_ACTIVATE);
    if (ret != HAL_OK) return ret;
    HAL_Delay(30u);
    return HAL_OK;
}

HAL_StatusTypeDef CVD_Deactivate(CVD_HandleTypeDef *cvd)
{
    HAL_StatusTypeDef ret = CVD_SendCommand(cvd, CVD_CMD_DEACTIVATE);
    if (ret != HAL_OK) return ret;
    HAL_Delay(30u);
    return HAL_OK;
}

HAL_StatusTypeDef CVD_ClearCommError(CVD_HandleTypeDef *cvd, uint16_t net_in_base)
{
    HAL_StatusTypeDef ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_NET_IN, net_in_base & (uint16_t)~CVD_NET_IN_ERR_CLR);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_NET_IN, net_in_base | CVD_NET_IN_ERR_CLR);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_NET_IN, net_in_base & (uint16_t)~CVD_NET_IN_ERR_CLR);
    return ret;
}

HAL_StatusTypeDef CVD_ClearAlarm(CVD_HandleTypeDef *cvd, uint16_t net_in_base)
{
    HAL_StatusTypeDef ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_NET_IN, net_in_base & (uint16_t)~CVD_NET_IN_ALM_RST);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_NET_IN, net_in_base | CVD_NET_IN_ALM_RST);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_NET_IN, net_in_base & (uint16_t)~CVD_NET_IN_ALM_RST);
    return ret;
}

HAL_StatusTypeDef CVD_EnableMotorPin(CVD_HandleTypeDef *cvd)
{
    if (cvd == NULL) return HAL_ERROR;
    if (cvd->enable_port != NULL) {
        HAL_GPIO_WritePin(cvd->enable_port, cvd->enable_pin, GPIO_PIN_RESET);
    }
    return HAL_OK;
}

HAL_StatusTypeDef CVD_DisableMotorPin(CVD_HandleTypeDef *cvd)
{
    if (cvd == NULL) return HAL_ERROR;
    if (cvd->enable_port != NULL) {
        HAL_GPIO_WritePin(cvd->enable_port, cvd->enable_pin, GPIO_PIN_SET);
    }
    return HAL_OK;
}

HAL_StatusTypeDef CVD_VerifyConfig(CVD_HandleTypeDef *cvd, const CVD_ConfigTypeDef *cfg)
{
    uint16_t value;
    if ((cvd == NULL) || (cfg == NULL)) return HAL_ERROR;
    if (CVD_ReadReg16(cvd, CVD_REG_RUN_CRNT, &value) != HAL_OK) return HAL_ERROR;
    if (value != cfg->run_current) return HAL_ERROR;
    if (CVD_ReadReg16(cvd, CVD_REG_STOP_CRNT, &value) != HAL_OK) return HAL_ERROR;
    if (value != cfg->stop_current) return HAL_ERROR;
    if (CVD_ReadReg16(cvd, CVD_REG_SETTING, &value) != HAL_OK) return HAL_ERROR;
    if ((value & CVD_SETTING_VERIFY_MASK) != (cfg->setting & CVD_SETTING_VERIFY_MASK)) return HAL_ERROR;
    if (CVD_ReadReg16(cvd, CVD_REG_RESOLUTION, &value) != HAL_OK) return HAL_ERROR;
    if (value != cfg->resolution) return HAL_ERROR;
    if (CVD_ReadReg16(cvd, CVD_REG_MOT_SEL, &value) != HAL_OK) return HAL_ERROR;
    if (value != cfg->mot_sel) return HAL_ERROR;
    if (CVD_ReadReg16(cvd, CVD_REG_NET_IN, &value) != HAL_OK) return HAL_ERROR;
    if ((value & CVD_NET_IN_VERIFY_MASK) != (cfg->net_in & CVD_NET_IN_VERIFY_MASK)) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef CVD_ApplyConfig(CVD_HandleTypeDef *cvd, const CVD_ConfigTypeDef *cfg)
{
    HAL_StatusTypeDef ret;
    if ((cvd == NULL) || (cfg == NULL)) return HAL_ERROR;
    ret = CVD_DisableMotorPin(cvd);
    if (ret != HAL_OK) return ret;
    ret = CVD_Deactivate(cvd);
    if (ret != HAL_OK) return ret;
    ret = CVD_ClearCommError(cvd, cfg->net_in);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_NET_IN, cfg->net_in);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_RUN_CRNT, cfg->run_current);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_STOP_CRNT, cfg->stop_current);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_SETTING, cfg->setting);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_RESOLUTION, cfg->resolution);
    if (ret != HAL_OK) return ret;
    ret = CVD_WriteReg16(cvd, CVD_REG_MOT_SEL, cfg->mot_sel);
    if (ret != HAL_OK) return ret;
    ret = CVD_VerifyConfig(cvd, cfg);
    if (ret != HAL_OK) return ret;
    ret = CVD_Activate(cvd);
    if (ret != HAL_OK) return ret;
    ret = CVD_WaitOperationState(cvd, 100u);
    if (ret != HAL_OK) return ret;
    return HAL_OK;
}

HAL_StatusTypeDef CVD_ApplyConfigAndEnable(CVD_HandleTypeDef *cvd, const CVD_ConfigTypeDef *cfg)
{
    HAL_StatusTypeDef ret = CVD_ApplyConfig(cvd, cfg);
    if (ret != HAL_OK) return ret;
    ret = CVD_EnableMotorPin(cvd);
    if (ret != HAL_OK) return ret;
    HAL_Delay(20u);
    return HAL_OK;
}

HAL_StatusTypeDef CVD_ReadNetOut(CVD_HandleTypeDef *cvd, uint16_t *net_out)
{
    return CVD_ReadReg16(cvd, CVD_REG_NET_OUT, net_out);
}

HAL_StatusTypeDef CVD_ReadAlarmCode(CVD_HandleTypeDef *cvd, uint16_t *alarm_code)
{
    return CVD_ReadReg16(cvd, CVD_REG_ALM_CD, alarm_code);
}

HAL_StatusTypeDef CVD_WaitOperationState(CVD_HandleTypeDef *cvd, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) <= timeout_ms) {
        uint16_t status0 = 0u;
        uint16_t status1 = 0u;
        if (CVD_Nop(cvd, &status0, &status1) != HAL_OK) return HAL_ERROR;
        if ((status1 & CVD_STATUS1_STATE_OPERATION) != 0u) return HAL_OK;
        HAL_Delay(2u);
    }
    return HAL_TIMEOUT;
}
