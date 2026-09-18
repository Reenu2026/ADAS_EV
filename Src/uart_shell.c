/**
 ******************************************************************************
 * @file    uart_shell.c
 * @brief   Module 4 - UART Shell & Python Dashboard Integration
 *          - USART1 @ 115200 baud, 8N1
 *          - 1 Hz telemetry TX of all integer-scaled EV/ADAS values
 *          - Interrupt-driven RX into a ring buffer, line-based command shell
 *            supporting: "status", "fault clear", "set speed <n>", "set mode <eco|normal|sport>"
 ******************************************************************************
 */
#include "uart_shell.h"
#include "ev_controller.h"
#include "adas_system.h"
#include "fsm_fault.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- RX Ring buffer ---- */
static volatile uint8_t  rxRing[UART_RX_RING_SIZE];
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;
uint8_t g_uartRxStagingByte; /* single byte staging area for HAL_UART_Receive_IT, referenced by main.c ISR */

static char cmdLine[UART_CMD_MAX_LEN];
static uint8_t cmdIndex = 0;

void UART_Shell_Init(void)
{
    rxHead = rxTail = 0;
    cmdIndex = 0;
    memset(cmdLine, 0, sizeof(cmdLine));

    /* Kick off the first interrupt-driven single-byte receive */
    HAL_UART_Receive_IT(&huart1, &g_uartRxStagingByte, 1);
}

/**
 * @brief Called from HAL_UART_RxCpltCallback(). Pushes the received byte
 *        into the ring buffer and re-arms the next single-byte receive.
 */
void UART_Shell_RxByteISR(uint8_t byte)
{
    uint16_t nextHead = (rxHead + 1) % UART_RX_RING_SIZE;
    if (nextHead != rxTail) { /* drop byte if buffer full */
        rxRing[rxHead] = byte;
        rxHead = nextHead;
    }
    HAL_UART_Receive_IT(&huart1, &g_uartRxStagingByte, 1); /* re-arm */
}

static void UART_Shell_SendString(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), 50);
}

/**
 * @brief Parses a complete command line and executes the corresponding action.
 */
static void UART_Shell_ExecuteCommand(char *line)
{
    /* Trim trailing CR/LF */
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
        line[--len] = '\0';
    }
    if (len == 0) return;

    if (strcmp(line, "status") == 0) {
        char buf[UART_TX_MAX_LEN];
        snprintf(buf, sizeof(buf),
                 "---- STATUS ----\r\n"
                 "STATE: %d\r\n"
                 "FAULT: 0x%02X\r\n"
                 "SPEED: %d\r\n"
                 "SOC: %d\r\n"
                 "TEMP: %d\r\n"
                 "TORQUE: %d\r\n"
                 "RANGE: %d\r\n"
                 "TTC: %d\r\n"
                 "FRONT: %d\r\n"
                 "LEFT: %d\r\n"
                 "RIGHT: %d\r\n"
                 "----------------\r\n",
                 (int)vehicleState, faultFlags,
                 (int)evData.speedKmh, (int)evData.soc, (int)evData.motorTempC,
                 (int)evData.torqueNm, (int)evData.rangeKm, (int)(adasData.ttcSeconds * 10),
                 (int)adasData.frontDistanceCm, (int)adasData.leftDistanceCm, (int)adasData.rightDistanceCm);
        UART_Shell_SendString(buf);
    }
    else if (strcmp(line, "fault clear") == 0) {
        FSM_ClearFault();
        UART_Shell_SendString("OK: fault cleared\r\n");
    }
    else if (strncmp(line, "set speed", 9) == 0) {
        /* Diagnostic override, e.g. "set speed 15" - clamps to max speed in EV module */
        char *arg = line + 9;
        while (*arg == ' ') arg++;
        int val = atoi(arg);
        if (val >= 0 && val <= 20) {
            evData.speedKmh = (float)val;
            UART_Shell_SendString("OK: speed set\r\n");
        } else {
            UART_Shell_SendString("ERR: speed out of range (0-20)\r\n");
        }
    }
    else if (strncmp(line, "set mode", 8) == 0) {
        char *arg = line + 8;
        while (*arg == ' ') arg++;
        if (strncmp(arg, "eco", 3) == 0)         { EV_SetDriveMode(MODE_ECO);    UART_Shell_SendString("OK: mode=eco\r\n"); }
        else if (strncmp(arg, "normal", 6) == 0) { EV_SetDriveMode(MODE_NORMAL); UART_Shell_SendString("OK: mode=normal\r\n"); }
        else if (strncmp(arg, "sport", 5) == 0)  { EV_SetDriveMode(MODE_SPORT);  UART_Shell_SendString("OK: mode=sport\r\n"); }
        else UART_Shell_SendString("ERR: unknown mode\r\n");
    }
    else {
        UART_Shell_SendString("ERR: unknown command\r\n");
    }
}

/**
 * @brief Drains the RX ring buffer, assembles line(s) terminated by '\n',
 *        and dispatches each completed line to the command executor.
 *        Call from the main super-loop.
 */
void UART_Shell_Process(void)
{
    while (rxTail != rxHead) {
        uint8_t c = rxRing[rxTail];
        rxTail = (rxTail + 1) % UART_RX_RING_SIZE;

        if (c == '\n') {
            cmdLine[cmdIndex] = '\0';
            UART_Shell_ExecuteCommand(cmdLine);
            cmdIndex = 0;
        } else if (c != '\r') {
            if (cmdIndex < (UART_CMD_MAX_LEN - 1)) {
                cmdLine[cmdIndex++] = (char)c;
            } else {
                cmdIndex = 0; /* overflow guard: reset line */
            }
        }
    }
}

/**
 * @brief Transmits one telemetry frame with all integer-scaled values,
 *        one field per line for easy reading in a terminal.
 *        Call once every 1000 ms from the main loop / a 1s software timer.
 */
void UART_Shell_TransmitTelemetry(void)
{
    char buf[UART_TX_MAX_LEN];
    uint8_t alarmByte = (adasData.criticalAlarm   ? 0x01 : 0) |
                        (adasData.warningAlarm    ? 0x02 : 0) |
                        (adasData.blindSpotLeft   ? 0x04 : 0) |
                        (adasData.blindSpotRight  ? 0x08 : 0);

    int len = snprintf(buf, sizeof(buf),
             "speed: %d\r\n"
             "soc: %d\r\n"
             "torque: %d\r\n"
             "range: %d\r\n"
             "temp: %d\r\n"
             "ttc: %d\r\n"
             "front: %d\r\n"
             "left: %d\r\n"
             "right: %d\r\n"
             "alarm: %d\r\n"
             "state: %d\r\n"
             "fault: %d\r\n"
             "----\r\n",
             (int)evData.speedKmh,
             (int)evData.soc,
             (int)evData.torqueNm,
             (int)evData.rangeKm,
             (int)evData.motorTempC,
             (int)(adasData.ttcSeconds * 10.0f),
             (int)adasData.frontDistanceCm,
             (int)adasData.leftDistanceCm,
             (int)adasData.rightDistanceCm,
             (int)alarmByte,
             (int)vehicleState,
             (int)faultFlags);

    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
}