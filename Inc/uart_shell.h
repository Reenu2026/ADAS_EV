#ifndef __UART_SHELL_H
#define __UART_SHELL_H

#include "main.h"

#define UART_RX_RING_SIZE   128
#define UART_CMD_MAX_LEN     64
#define UART_TX_MAX_LEN     160

void UART_Shell_Init(void);

/* Called from HAL_UART_RxCpltCallback ISR - pushes 1 byte into ring buffer */
void UART_Shell_RxByteISR(uint8_t byte);

/* Called from main loop - drains ring buffer, assembles lines, parses commands */
void UART_Shell_Process(void);

/* Transmits the full telemetry line (Speed, SOC, Torque, Range, Temp, TTC,
 * distances, alarm flags) as an integer-string message. Call once per second. */
void UART_Shell_TransmitTelemetry(void);

#endif /* __UART_SHELL_H */
