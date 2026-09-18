/**
 ******************************************************************************
 * @file    main.h
 * @brief   Real-time EV & ADAS Dashboard - STM32F103C8T6 (Blue Pill)
 *          Simulated on PiximLab, built in STM32CubeIDE using HAL drivers.
 ******************************************************************************
 */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ==================== GPIO PIN MAP ==================== */

/* ---- ADC1 Channels (EV potentiometers) ---- */
#define ADC_ACCEL_PORT      GPIOA
#define ADC_ACCEL_PIN       GPIO_PIN_0   /* PA0 - Acceleration Pedal   */
#define ADC_BRAKE_PORT      GPIOA
#define ADC_BRAKE_PIN       GPIO_PIN_1   /* PA1 - Brake Pedal          */
#define ADC_SOC_PORT        GPIOA
#define ADC_SOC_PIN         GPIO_PIN_2   /* PA2 - Initial SOC          */
#define ADC_TEMP_PORT       GPIOA
#define ADC_TEMP_PIN        GPIO_PIN_3   /* PA3 - Motor Temperature    */

/* ---- Ultrasonic Sensors (ADAS) ---- */
#define TRIG_FRONT_PORT     GPIOB
#define TRIG_FRONT_PIN      GPIO_PIN_0   /* PB0 - Front Trigger */
#define ECHO_FRONT_PORT     GPIOB
#define ECHO_FRONT_PIN      GPIO_PIN_1   /* PB1 - Front Echo    */

#define TRIG_LEFT_PORT      GPIOB
#define TRIG_LEFT_PIN       GPIO_PIN_2   /* PB2 - Left Trigger  */
#define ECHO_LEFT_PORT      GPIOB
#define ECHO_LEFT_PIN       GPIO_PIN_3   /* PB3 - Left Echo     */

#define TRIG_RIGHT_PORT     GPIOB
#define TRIG_RIGHT_PIN      GPIO_PIN_4   /* PB4 - Right Trigger */
#define ECHO_RIGHT_PORT     GPIOB
#define ECHO_RIGHT_PIN      GPIO_PIN_5   /* PB5 - Right Echo    */

/* ---- LED Indicators ---- */
#define LED_HEARTBEAT_PORT  GPIOC
#define LED_HEARTBEAT_PIN   GPIO_PIN_13  /* PC13 - Onboard heartbeat LED   */

#define LED_RED_PORT        GPIOB
#define LED_RED_PIN         GPIO_PIN_8   /* PB8  - Critical collision warn */

#define LED_BLUE_PORT       GPIOB
#define LED_BLUE_PIN        GPIO_PIN_9   /* PB9  - Left blind spot         */

#define LED_GREEN_PORT      GPIOB
#define LED_GREEN_PIN       GPIO_PIN_10  /* PB10 - Right blind spot        */

#define LED_YELLOW_PORT     GPIOB
#define LED_YELLOW_PIN      GPIO_PIN_11  /* PB11 - System fault            */

/* ==================== GLOBAL HANDLES ==================== */
extern ADC_HandleTypeDef   hadc1;
extern TIM_HandleTypeDef   htim1;   /* 10ms  - EV sampling      */
extern TIM_HandleTypeDef   htim2;   /* 1MHz  - Echo pulse timer */
extern TIM_HandleTypeDef   htim3;   /* 100ms - ADAS sampling    */
extern UART_HandleTypeDef  huart1;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
