/**
 ******************************************************************************
 * @file    main.c
 * @brief   Real-time EV & ADAS Dashboard
 *          STM32F103C8T6 "Blue Pill" (simulated in PiximLab), STM32CubeIDE / HAL.
 *
 *          System clock : HSE 8MHz -> PLL x9 -> 72 MHz
 *          TIM1  : 10ms  interrupt -> EV pedal/SOC/temp sampling (Module 1)
 *          TIM2  : free-running 1MHz counter (PSC=71) -> ultrasonic echo timing
 *          TIM3  : 100ms interrupt -> ADAS ultrasonic scan (Module 2)
 *          USART1: 115200 baud -> telemetry TX (1Hz) + shell RX (Module 4)
 ******************************************************************************
 */
#include "main.h"
#include "ev_controller.h"
#include "adas_system.h"
#include "fsm_fault.h"
#include "uart_shell.h"

/* ==================== Peripheral Handles ==================== */
ADC_HandleTypeDef   hadc1;
TIM_HandleTypeDef   htim1;
TIM_HandleTypeDef   htim2;
TIM_HandleTypeDef   htim3;
UART_HandleTypeDef  huart1;

/* ==================== Scheduler flags (set in ISRs, serviced in main loop) ==================== */
static volatile uint8_t flag_10ms_EV      = 0;
static volatile uint8_t flag_100ms_ADAS   = 0;
static volatile uint32_t ms_counter_1s    = 0;
static volatile uint8_t flag_1000ms_UART  = 0;

/* ==================== Forward declarations ==================== */
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void ADC1_Init(void);
static void TIM1_Init(void);   /* 10 ms  */
static void TIM2_Init(void);   /* 1 MHz free-running */
static void TIM3_Init(void);   /* 100 ms */
static void USART1_UART_Init(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    GPIO_Init();
    ADC1_Init();
    TIM1_Init();
    TIM2_Init();
    TIM3_Init();
    USART1_UART_Init();

    EV_Init();
    ADAS_Init();
    FSM_Init();
    UART_Shell_Init();

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start(&htim2);      /* free-running, no interrupt needed */
    HAL_TIM_Base_Start_IT(&htim3);

    while (1)
    {
        /* ---- 10 ms: EV controller update (Module 1) ---- */
        if (flag_10ms_EV) {
            flag_10ms_EV = 0;
            EV_ReadPedalsAndSOC();
            EV_ProcessStep(0.010f);
            FSM_Update();   /* re-evaluate FSM + faults every 10ms tick */
        }

        /* ---- 100 ms: ADAS ultrasonic scan (Module 2) ---- */
        if (flag_100ms_ADAS) {
            flag_100ms_ADAS = 0;
            ADAS_ScanSensors(evData.speedKmh);
        }

        /* ---- 1000 ms: telemetry TX (Module 4) ---- */
        if (flag_1000ms_UART) {
            flag_1000ms_UART = 0;
            UART_Shell_TransmitTelemetry();
        }

        /* ---- Continuous: drain UART RX ring buffer / shell commands ---- */
        UART_Shell_Process();
    }
}

/* ==================== Timer ISR Callback ==================== */
/**
 * HAL_TIM_PeriodElapsedCallback is shared across all TIM instances using
 * update-event interrupts. We branch on htim->Instance to dispatch the
 * correct scheduler flag - this is the "cooperative scheduler" described
 * in the spec (Section 3).
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        flag_10ms_EV = 1;

        /* Heartbeat LED toggles roughly every 500ms (50 x 10ms ticks) */
        static uint8_t hbCount = 0;
        if (++hbCount >= 50) {
            hbCount = 0;
            HAL_GPIO_TogglePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
        }

        /* 1-second software timer built on top of the 10ms tick (100 x 10ms) */
        ms_counter_1s++;
        if (ms_counter_1s >= 100) {
            ms_counter_1s = 0;
            flag_1000ms_UART = 1;
        }
    }
    else if (htim->Instance == TIM3) {
        flag_100ms_ADAS = 1;
    }
}

/* ==================== UART RX Complete Callback ==================== */
/* uart_shell.c owns the single-byte staging variable (g_uartRxStagingByte)
 * and re-arms the next HAL_UART_Receive_IT() call inside UART_Shell_RxByteISR().
 * We just forward the received byte to the shell's ring buffer here. */
extern uint8_t g_uartRxStagingByte; /* defined in uart_shell.c */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        UART_Shell_RxByteISR(g_uartRxStagingByte);
    }
}

/* ==================== Clock Configuration ==================== */
/**
 * @brief System Clock Configuration
 *        HSE (8MHz crystal) -> PLL (x9) -> SYSCLK 72MHz
 *        AHB = 72MHz, APB1 = 36MHz, APB2 = 72MHz
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;   /* 8MHz x 9 = 72MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_PCLK1   | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* 36 MHz (max for APB1) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* 72 MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* ==================== GPIO Init ==================== */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* ---- Outputs: Trigger pins + LEDs ---- */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = TRIG_FRONT_PIN | TRIG_LEFT_PIN | TRIG_RIGHT_PIN |
                          LED_RED_PIN | LED_BLUE_PIN | LED_GREEN_PIN | LED_YELLOW_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_HEARTBEAT_PIN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ---- Inputs: Echo pins ---- */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin  = ECHO_FRONT_PIN | ECHO_LEFT_PIN | ECHO_RIGHT_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ---- Analog inputs for ADC1 (PA0-PA3) ---- */
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin  = ADC_ACCEL_PIN | ADC_BRAKE_PIN | ADC_SOC_PIN | ADC_TEMP_PIN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Initial output states: all off */
    HAL_GPIO_WritePin(GPIOB, TRIG_FRONT_PIN | TRIG_LEFT_PIN | TRIG_RIGHT_PIN |
                              LED_RED_PIN | LED_BLUE_PIN | LED_GREEN_PIN | LED_YELLOW_PIN,
                       GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, LED_HEARTBEAT_PIN, GPIO_PIN_SET); /* onboard LED often active-low */
}

/* ==================== ADC1 Init (4 channels, software-triggered polling) ==================== */
static void ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* Run the ADC calibration routine (STM32F1 specific) */
    HAL_ADCEx_Calibration_Start(&hadc1);
}

/* ==================== TIM1: 10 ms periodic interrupt ==================== */
/* APB2 timer clock = 72MHz. PSC=7199 -> 10kHz tick, ARR=99 -> 10ms period */
static void TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler         = 7199;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 99;    /* 10ms */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

/* ==================== TIM2: free-running 1MHz counter (PSC=71) ==================== */
/* APB1 timer clock = 72MHz (x2 since APB1 prescaler !=1). PSC=71 -> 1MHz (1us tick) */
static void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler     = 71;         /* 72MHz / (71+1) = 1 MHz */
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period        = 0xFFFF;     /* free-running 16-bit counter */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    /* No interrupt required - polled via __HAL_TIM_GET_COUNTER() in adas_system.c */
}

/* ==================== TIM3: 100 ms periodic interrupt ==================== */
/* APB1 timer clock = 72MHz. PSC=7199 -> 10kHz tick, ARR=999 -> 100ms period */
static void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler     = 7199;
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period        = 999;   /* 100ms */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ==================== USART1: 115200 baud ==================== */
static void USART1_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* PA9 = TX (AF push-pull), PA10 = RX (floating input) */
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ==================== Error Handler ==================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(LED_YELLOW_PORT, LED_YELLOW_PIN);
        for (volatile uint32_t i = 0; i < 500000; i++);
    }
}
