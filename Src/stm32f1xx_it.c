/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines - wires vector table entries to the
 *          HAL's generic IRQ handlers, which in turn invoke the
 *          HAL_TIM_PeriodElapsedCallback / HAL_UART_RxCpltCallback etc.
 *          defined in main.c and uart_shell.c.
 ******************************************************************************
 */
#include "main.h"

extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

void NMI_Handler(void)        { while (1) {} }
void HardFault_Handler(void)  { while (1) {} }
void MemManage_Handler(void)  { while (1) {} }
void BusFault_Handler(void)   { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* TIM1 update (overflow) interrupt - 10ms EV sampling tick */
void TIM1_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

/* TIM3 global interrupt - 100ms ADAS sampling tick */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

/* USART1 global interrupt - shell RX */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
