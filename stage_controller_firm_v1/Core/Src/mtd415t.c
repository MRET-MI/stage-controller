/*
 * mtd415t.c
 *
 *  Thorlabs MTD415T TEC temperature-controller driver (UART bridge).
 *  See mtd415t.h for the link/protocol overview and the full command list.
 *
 *  Created on: 2026/06/06
 */

#include "mtd415t.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── RX ring buffer ────────────────────────────────────────────────────────── */

#define MTD_RX_RING  256u

static UART_HandleTypeDef *s_huart  = NULL;
static volatile uint8_t    s_ring[MTD_RX_RING];
static volatile uint16_t   s_head   = 0u;   /* written by ISR  */
static volatile uint16_t   s_tail   = 0u;   /* read by main    */
static          uint8_t    s_rx_byte;       /* HAL 1-byte landing slot */

/* ── Lifecycle ─────────────────────────────────────────────────────────────── */

void MTD415T_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_head  = 0u;
    s_tail  = 0u;
    if (s_huart != NULL) {
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

/* RX-complete: push the received byte and re-arm 1-byte reception.
 * No other UART is used in this project, so a single global callback is safe. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart && s_huart != NULL) {
        uint16_t next = (uint16_t)((s_head + 1u) % MTD_RX_RING);
        if (next != s_tail) {            /* drop byte on overflow rather than overwrite */
            s_ring[s_head] = s_rx_byte;
            s_head = next;
        }
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

/* Re-arm reception if an error (ORE/FE/NE) aborted the IT receive. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart && s_huart != NULL) {
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

int MTD415T_RxReadByte(uint8_t *b)
{
    if (s_tail == s_head) {
        return 0;
    }
    *b = s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1u) % MTD_RX_RING);
    return 1;
}

/* Discard any pending bytes (stale responses) before a new query. */
static void MTD415T_RxFlush(void)
{
    s_tail = s_head;
}

/* ── Low-level link ────────────────────────────────────────────────────────── */

void MTD415T_SendRaw(const char *cmd)
{
    if (s_huart == NULL || cmd == NULL) {
        return;
    }
    uint8_t lf = (uint8_t)'\n';
    HAL_UART_Transmit(s_huart, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100u);
    HAL_UART_Transmit(s_huart, &lf, 1u, 10u);
}

int MTD415T_Query(const char *cmd, char *resp, size_t resp_size, uint32_t timeout_ms)
{
    if (resp == NULL || resp_size == 0u) {
        return -1;
    }
    resp[0] = '\0';

    MTD415T_RxFlush();
    MTD415T_SendRaw(cmd);

    uint32_t start = HAL_GetTick();
    size_t   i     = 0u;
    uint8_t  b;

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (MTD415T_RxReadByte(&b)) {
            if (b == (uint8_t)'\r') {
                continue;
            }
            if (b == (uint8_t)'\n') {
                resp[i] = '\0';
                return (int)i;          /* complete line */
            }
            if (i < (resp_size - 1u)) {
                resp[i++] = (char)b;
            }
        }
    }
    resp[i] = '\0';
    return -1;                          /* timeout */
}

/* ── Generic typed helpers ─────────────────────────────────────────────────── */

void MTD415T_SetValue(char letter, long value)
{
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "%c%ld", letter, value);
    MTD415T_SendRaw(cmd);
}

int MTD415T_QueryLong(const char *query, long *out)
{
    char resp[32];
    if (MTD415T_Query(query, resp, sizeof(resp), MTD415T_QUERY_TIMEOUT_MS) < 0) {
        return -1;
    }
    if (out != NULL) {
        *out = strtol(resp, NULL, 10);
    }
    return 0;
}

/* ── General commands ──────────────────────────────────────────────────────── */

int MTD415T_GetVersion(char *resp, size_t resp_size)
{
    return (MTD415T_Query("m?", resp, resp_size, MTD415T_QUERY_TIMEOUT_MS) < 0) ? -1 : 0;
}

int MTD415T_GetUUID(char *resp, size_t resp_size)
{
    return (MTD415T_Query("u?", resp, resp_size, MTD415T_QUERY_TIMEOUT_MS) < 0) ? -1 : 0;
}

int MTD415T_GetError(uint32_t *err)
{
    char resp[32];
    if (MTD415T_Query("E?", resp, sizeof(resp), MTD415T_QUERY_TIMEOUT_MS) < 0) {
        return -1;
    }
    if (err != NULL) {
        *err = (uint32_t)strtoul(resp, NULL, 0);
    }
    return 0;
}

void MTD415T_ClearError(void) { MTD415T_SendRaw("c"); }
void MTD415T_Save(void)       { MTD415T_SendRaw("M"); }

/* ── Temperature commands ──────────────────────────────────────────────────── */

void MTD415T_SetTempMdegC(int32_t mdegC)      { MTD415T_SetValue('T', (long)mdegC); }

int  MTD415T_GetSetTempMdegC(int32_t *mdegC)
{
    long v;
    if (MTD415T_QueryLong("T?", &v) < 0) { return -1; }
    if (mdegC != NULL) { *mdegC = (int32_t)v; }
    return 0;
}

int  MTD415T_GetActualTempMdegC(int32_t *mdegC)
{
    long v;
    if (MTD415T_QueryLong("Te?", &v) < 0) { return -1; }
    if (mdegC != NULL) { *mdegC = (int32_t)v; }
    return 0;
}

void MTD415T_SetWindowmK(int32_t mK)          { MTD415T_SetValue('W', (long)mK); }

int  MTD415T_GetWindowmK(int32_t *mK)
{
    long v;
    if (MTD415T_QueryLong("W?", &v) < 0) { return -1; }
    if (mK != NULL) { *mK = (int32_t)v; }
    return 0;
}

void MTD415T_SetWindowDelaySec(int32_t sec)   { MTD415T_SetValue('d', (long)sec); }

int  MTD415T_GetWindowDelaySec(int32_t *sec)
{
    long v;
    if (MTD415T_QueryLong("d?", &v) < 0) { return -1; }
    if (sec != NULL) { *sec = (int32_t)v; }
    return 0;
}

/* ── TEC commands ──────────────────────────────────────────────────────────── */

void MTD415T_SetCurrentLimitmA(int32_t mA)    { MTD415T_SetValue('L', (long)mA); }

int  MTD415T_GetCurrentLimitmA(int32_t *mA)
{
    long v;
    if (MTD415T_QueryLong("L?", &v) < 0) { return -1; }
    if (mA != NULL) { *mA = (int32_t)v; }
    return 0;
}

int  MTD415T_GetCurrentmA(int32_t *mA)
{
    long v;
    if (MTD415T_QueryLong("A?", &v) < 0) { return -1; }
    if (mA != NULL) { *mA = (int32_t)v; }
    return 0;
}

int  MTD415T_GetVoltagemV(int32_t *mV)
{
    long v;
    if (MTD415T_QueryLong("U?", &v) < 0) { return -1; }
    if (mV != NULL) { *mV = (int32_t)v; }
    return 0;
}

/* ── Control-loop (PID) commands ───────────────────────────────────────────── */

void MTD415T_SetCriticalGain(int32_t g)       { MTD415T_SetValue('G', (long)g); }
int  MTD415T_GetCriticalGain(int32_t *g)
{
    long v; if (MTD415T_QueryLong("G?", &v) < 0) { return -1; }
    if (g != NULL) { *g = (int32_t)v; } return 0;
}

void MTD415T_SetCriticalPeriodMs(int32_t ms)  { MTD415T_SetValue('O', (long)ms); }
int  MTD415T_GetCriticalPeriodMs(int32_t *ms)
{
    long v; if (MTD415T_QueryLong("O?", &v) < 0) { return -1; }
    if (ms != NULL) { *ms = (int32_t)v; } return 0;
}

void MTD415T_SetCyclingTimeMs(int32_t ms)     { MTD415T_SetValue('C', (long)ms); }
int  MTD415T_GetCyclingTimeMs(int32_t *ms)
{
    long v; if (MTD415T_QueryLong("C?", &v) < 0) { return -1; }
    if (ms != NULL) { *ms = (int32_t)v; } return 0;
}

void MTD415T_SetPShare(int32_t p)             { MTD415T_SetValue('P', (long)p); }
int  MTD415T_GetPShare(int32_t *p)
{
    long v; if (MTD415T_QueryLong("P?", &v) < 0) { return -1; }
    if (p != NULL) { *p = (int32_t)v; } return 0;
}

void MTD415T_SetIShare(int32_t i)             { MTD415T_SetValue('I', (long)i); }
int  MTD415T_GetIShare(int32_t *i)
{
    long v; if (MTD415T_QueryLong("I?", &v) < 0) { return -1; }
    if (i != NULL) { *i = (int32_t)v; } return 0;
}

void MTD415T_SetDShare(int32_t d)             { MTD415T_SetValue('D', (long)d); }
int  MTD415T_GetDShare(int32_t *d)
{
    long v; if (MTD415T_QueryLong("D?", &v) < 0) { return -1; }
    if (d != NULL) { *d = (int32_t)v; } return 0;
}
