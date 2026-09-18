/**
 ******************************************************************************
 * @file    fsm_fault.c
 * @brief   Module 3 - Finite State Machine & Fault Manager
 *          States: Parked -> Ready -> Driving <-> Regeneration, with an
 *          instant override to Fault whenever a critical condition is seen.
 ******************************************************************************
 */
#include "fsm_fault.h"
#include "ev_controller.h"
#include "adas_system.h"

VehicleState_t vehicleState = STATE_PARKED;
uint8_t        faultFlags   = FAULT_NONE;

#define OVERHEAT_TEMP_C     120u
#define LOW_BATTERY_SOC       5u
#define ACCEL_START_PCT       2u
#define BRAKE_REGEN_PCT       5u

void FSM_Init(void)
{
    vehicleState = STATE_PARKED;
    faultFlags   = FAULT_NONE;
}

void FSM_ClearFault(void)
{
    faultFlags = FAULT_NONE;
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET);
    vehicleState = STATE_READY;
}

static void FSM_CheckFaults(void)
{
    faultFlags = FAULT_NONE;

    if (evData.motorTempC > OVERHEAT_TEMP_C) {
        faultFlags |= FAULT_OVERHEAT;
    }
    if (evData.soc < LOW_BATTERY_SOC) {
        faultFlags |= FAULT_LOW_BATTERY;
    }
    if (adasData.criticalAlarm) {
        faultFlags |= FAULT_COLLISION;
    }

    if (faultFlags != FAULT_NONE) {
        vehicleState = STATE_FAULT;
    }
}

void FSM_Update(void)
{
    /* Fault detection has top priority and can override any state instantly */
    FSM_CheckFaults();

    switch (vehicleState) {

        case STATE_PARKED:
            /* Wait for external "ready" trigger - simplified: auto-advance */
            vehicleState = STATE_READY;
            break;

        case STATE_READY:
            if (evData.accelPct > ACCEL_START_PCT) {
                vehicleState = STATE_DRIVING;
            }
            break;

        case STATE_DRIVING:
            if (evData.brakePct > BRAKE_REGEN_PCT) {
                vehicleState = STATE_REGEN;
            }
            break;

        case STATE_REGEN:
            if (evData.brakePct < BRAKE_REGEN_PCT) {
                vehicleState = STATE_DRIVING;
            }
            break;

        case STATE_FAULT:
            /* Force motor torque to zero - vehicle stopped */
            evData.torqueNm = 0.0f;
            HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_SET);
            /* Stays in FAULT until "fault clear" command is received via UART shell */
            break;

        default:
            vehicleState = STATE_FAULT;
            break;
    }

    if (vehicleState != STATE_FAULT) {
        HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET);
    }
}
