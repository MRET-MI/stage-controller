/*
 * mtd415t.h
 *
 *  Thorlabs MTD415T TEC temperature-controller driver (UART bridge).
 *
 *  Link layer : UART 115200 baud, 8-N-1, 3.3 V logic (direct connect, no
 *               level shifter). MTD415T-TXD -> STM32 RX, MTD415T-RXD -> STM32 TX.
 *  Protocol   : ASCII text. Set = "<letter><value>", read = "<letter>?".
 *               Both command and response are terminated by LF ('\n').
 *
 *  RX is captured by an interrupt-driven ring buffer (see HAL_UART_RxCpltCallback
 *  in mtd415t.c); MTD415T_Query() assembles one LF-terminated response line.
 *
 *  Full command set (every documented MTD415T UART command is wrapped below):
 *    General : m? (version), u? (UUID), E? (error reg), c (clear error), M (save)
 *    Temp    : T (setpoint m°C), Te? (measured m°C), W (window mK), d (window delay s)
 *    TEC     : L (current limit mA), A? (current mA), U? (voltage mV)
 *    Loop    : G (critical gain), O (critical period ms), C (cycling time ms),
 *              P (P share), I (I share), D (D share)
 *
 *  Created on: 2026/06/06
 */

#ifndef INC_MTD415T_H_
#define INC_MTD415T_H_

#include "main.h"        /* stm32h7xx_hal.h -> UART_HandleTypeDef */
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default timeout for a query/response round-trip [ms]. */
#define MTD415T_QUERY_TIMEOUT_MS  100u

/* ── Lifecycle ─────────────────────────────────────────────────────────────── */

/* Bind the UART handle and start interrupt RX. Call once after MX_USART3_UART_Init(). */
void MTD415T_Init(UART_HandleTypeDef *huart);

/* ── Low-level link (covers every command verbatim) ────────────────────────── */

/* Send a raw command string; a trailing LF is appended automatically. */
void MTD415T_SendRaw(const char *cmd);

/* Send cmd then read one LF-terminated response line into resp (NUL-terminated).
 * Returns response length (>=0) on success, or -1 on timeout. */
int  MTD415T_Query(const char *cmd, char *resp, size_t resp_size, uint32_t timeout_ms);

/* Pop one byte from the RX ring buffer. Returns 1 if a byte was read, else 0. */
int  MTD415T_RxReadByte(uint8_t *b);

/* ── Generic typed helpers ─────────────────────────────────────────────────── */

/* Send "<letter><value>" (set command). */
void MTD415T_SetValue(char letter, long value);
/* Send "<letter>?" and parse the integer reply. Returns 0 on success, -1 on error. */
int  MTD415T_QueryLong(const char *query, long *out);

/* ── General commands ──────────────────────────────────────────────────────── */

int  MTD415T_GetVersion(char *resp, size_t resp_size);   /* m?  */
int  MTD415T_GetUUID(char *resp, size_t resp_size);      /* u?  */
int  MTD415T_GetError(uint32_t *err);                    /* E?  */
void MTD415T_ClearError(void);                           /* c   */
void MTD415T_Save(void);                                 /* M   */

/* ── Temperature commands ──────────────────────────────────────────────────── */

void MTD415T_SetTempMdegC(int32_t mdegC);                /* Tx   set temperature [m°C] */
int  MTD415T_GetSetTempMdegC(int32_t *mdegC);            /* T?                          */
int  MTD415T_GetActualTempMdegC(int32_t *mdegC);         /* Te?  measured temp [m°C]    */
void MTD415T_SetWindowmK(int32_t mK);                    /* Wx   temp window [mK]       */
int  MTD415T_GetWindowmK(int32_t *mK);                   /* W?                          */
void MTD415T_SetWindowDelaySec(int32_t sec);             /* dx   window delay [s]       */
int  MTD415T_GetWindowDelaySec(int32_t *sec);            /* d?                          */

/* ── TEC commands ──────────────────────────────────────────────────────────── */

void MTD415T_SetCurrentLimitmA(int32_t mA);              /* Lx   limit [mA] 200..2000   */
int  MTD415T_GetCurrentLimitmA(int32_t *mA);             /* L?                          */
int  MTD415T_GetCurrentmA(int32_t *mA);                  /* A?   actual current [mA]    */
int  MTD415T_GetVoltagemV(int32_t *mV);                  /* U?   actual voltage [mV]    */

/* ── Control-loop (PID) commands ───────────────────────────────────────────── */

void MTD415T_SetCriticalGain(int32_t mA_per_K);          /* Gx  [mA/K]  10..100000      */
int  MTD415T_GetCriticalGain(int32_t *mA_per_K);         /* G?                          */
void MTD415T_SetCriticalPeriodMs(int32_t ms);            /* Ox  [ms]   100..100000      */
int  MTD415T_GetCriticalPeriodMs(int32_t *ms);           /* O?                          */
void MTD415T_SetCyclingTimeMs(int32_t ms);               /* Cx  [ms]     1..1000        */
int  MTD415T_GetCyclingTimeMs(int32_t *ms);              /* C?                          */
void MTD415T_SetPShare(int32_t p);                       /* Px  [mA/K]    0..100000     */
int  MTD415T_GetPShare(int32_t *p);                      /* P?                          */
void MTD415T_SetIShare(int32_t i);                       /* Ix  [mA/(K·s)] 0..100000    */
int  MTD415T_GetIShare(int32_t *i);                      /* I?                          */
void MTD415T_SetDShare(int32_t d);                       /* Dx  [(mA·s)/K] 0..100000    */
int  MTD415T_GetDShare(int32_t *d);                      /* D?                          */

#ifdef __cplusplus
}
#endif

#endif /* INC_MTD415T_H_ */
