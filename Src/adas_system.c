/**
 ******************************************************************************
 * @file    adas_system.c
 * @brief   Module 2 - ADAS Warning System
 *          Runs every 100 ms (TIM3 interrupt). Sequentially triggers the
 *          three ultrasonic sensors, measures echo pulse width with TIM2
 *          (1 MHz / 1us tick), computes distance, TTC, and drives the
 *          collision / blind-spot LEDs.
 ******************************************************************************
 */
#include "adas_system.h"

ADASData_t adasData;

#define SPEED_OF_SOUND_CM_PER_US   0.0343f   /* cm per microsecond */
#define ECHO_TIMEOUT_US            30000UL   /* ~5m round trip safety timeout */
#define NO_OBSTACLE_CM             200.0f
#define DEFAULT_TTC_S              99.9f

#define CRIT_DIST_CM               20.0f
#define CRIT_TTC_S                  1.5f
#define WARN_DIST_CM               50.0f
#define WARN_TTC_S                  3.0f
#define BLINDSPOT_DIST_CM          30.0f
#define BLINDSPOT_SPEED_KMH        10.0f   /* lowered from 20 - original threshold was unreachable given the 20km/h speed cap in ev_controller.c */

void ADAS_Init(void)
{
    adasData.frontDistanceCm = NO_OBSTACLE_CM;
    adasData.leftDistanceCm  = NO_OBSTACLE_CM;
    adasData.rightDistanceCm = NO_OBSTACLE_CM;
    adasData.ttcSeconds      = DEFAULT_TTC_S;
    adasData.criticalAlarm   = 0;
    adasData.warningAlarm    = 0;
    adasData.blindSpotLeft   = 0;
    adasData.blindSpotRight  = 0;

    HAL_GPIO_WritePin(TRIG_FRONT_PORT, TRIG_FRONT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG_LEFT_PORT,  TRIG_LEFT_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG_RIGHT_PORT, TRIG_RIGHT_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Fires a single ultrasonic sensor and measures echo width using
 *        TIM2 (already configured as free-running 1MHz counter, prescaler 71).
 * @return distance in centimeters, or NO_OBSTACLE_CM if timeout.
 */
static float ADAS_MeasureDistance(GPIO_TypeDef *trigPort, uint16_t trigPin,
                                   GPIO_TypeDef *echoPort, uint16_t echoPin)
{
    uint32_t startTick, endTick, elapsedUs;
    uint32_t waitStart;

    /* 10us trigger pulse */
    HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_SET);
    while (__HAL_TIM_GET_COUNTER(&htim2) < 10) { /* busy-wait 10us */ }
    HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);

    /* Wait for echo to go HIGH (with timeout) */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    waitStart = __HAL_TIM_GET_COUNTER(&htim2);
    while (HAL_GPIO_ReadPin(echoPort, echoPin) == GPIO_PIN_RESET) {
        if ((__HAL_TIM_GET_COUNTER(&htim2) - waitStart) > ECHO_TIMEOUT_US) {
            return NO_OBSTACLE_CM;
        }
    }

    /* Echo is HIGH: start timing */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    startTick = __HAL_TIM_GET_COUNTER(&htim2);

    while (HAL_GPIO_ReadPin(echoPort, echoPin) == GPIO_PIN_SET) {
        if ((__HAL_TIM_GET_COUNTER(&htim2) - startTick) > ECHO_TIMEOUT_US) {
            return NO_OBSTACLE_CM;
        }
    }
    endTick = __HAL_TIM_GET_COUNTER(&htim2);

    elapsedUs = (endTick >= startTick) ? (endTick - startTick)
                                        : (0xFFFFFFFFUL - startTick + endTick);

    float distanceCm = ((float)elapsedUs * SPEED_OF_SOUND_CM_PER_US) / 2.0f;
    if (distanceCm > NO_OBSTACLE_CM || distanceCm <= 0.0f) {
        distanceCm = NO_OBSTACLE_CM;
    }
    return distanceCm;
}

/**
 * @brief Sequentially scans front/left/right sensors, computes TTC and
 *        updates all alarm/LED states. Called every 100ms from TIM3 ISR.
 * @param currentSpeedKmh: latest vehicle speed from the EV controller.
 */
void ADAS_ScanSensors(float currentSpeedKmh)
{
    adasData.frontDistanceCm = ADAS_MeasureDistance(TRIG_FRONT_PORT, TRIG_FRONT_PIN,
                                                      ECHO_FRONT_PORT, ECHO_FRONT_PIN);
    adasData.leftDistanceCm  = ADAS_MeasureDistance(TRIG_LEFT_PORT, TRIG_LEFT_PIN,
                                                      ECHO_LEFT_PORT, ECHO_LEFT_PIN);
    adasData.rightDistanceCm = ADAS_MeasureDistance(TRIG_RIGHT_PORT, TRIG_RIGHT_PIN,
                                                      ECHO_RIGHT_PORT, ECHO_RIGHT_PIN);

    /* ---- Time To Collision ---- */
    if (adasData.frontDistanceCm >= NO_OBSTACLE_CM || currentSpeedKmh <= 0.01f) {
        adasData.ttcSeconds = DEFAULT_TTC_S;
    } else {
        float distM   = adasData.frontDistanceCm / 100.0f;
        float speedMs = currentSpeedKmh / 3.6f;
        adasData.ttcSeconds = distM / speedMs;
    }

    /* ---- Critical & Warning alarms (front collision) ---- */
    adasData.criticalAlarm = (adasData.frontDistanceCm < CRIT_DIST_CM ||
                               adasData.ttcSeconds < CRIT_TTC_S) ? 1 : 0;

    adasData.warningAlarm  = (adasData.frontDistanceCm < WARN_DIST_CM ||
                               adasData.ttcSeconds < WARN_TTC_S) ? 1 : 0;

    /* ---- Blind spot advisories ---- */
    adasData.blindSpotLeft  = (currentSpeedKmh > BLINDSPOT_SPEED_KMH &&
                                adasData.leftDistanceCm < BLINDSPOT_DIST_CM) ? 1 : 0;

    adasData.blindSpotRight = (currentSpeedKmh > BLINDSPOT_SPEED_KMH &&
                                adasData.rightDistanceCm < BLINDSPOT_DIST_CM) ? 1 : 0;

    /* ---- Drive LEDs ---- */
    HAL_GPIO_WritePin(LED_RED_PORT,   LED_RED_PIN,   adasData.criticalAlarm ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_BLUE_PORT,  LED_BLUE_PIN,  adasData.blindSpotLeft ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, adasData.blindSpotRight ? GPIO_PIN_SET : GPIO_PIN_RESET);
}