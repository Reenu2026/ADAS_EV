/**
 ******************************************************************************
 * @file    ev_controller.c
 * @brief   Module 1 - EV Controller Logic
 *          Runs every 10 ms (TIM1 interrupt). Converts raw ADC counts into
 *          pedal/SOC/temperature values and applies a simple physics-based
 *          inertia model to derive torque, acceleration, speed and range.
 ******************************************************************************
 */
#include "ev_controller.h"
#include <math.h>

EVData_t evData;

/* ---- Simulation constants ---- */
#define VEHICLE_MASS_KG        800.0f
#define DRAG_COEFF              0.35f
#define MAX_SPEED_KMH           20.0f   /* capped per spec */
#define MAX_MOTOR_TORQUE_NM    150.0f
#define REGEN_TORQUE_GAIN        1.2f
#define BRAKE_THRESHOLD_PCT       5u

/* Per-mode torque scaling & energy consumption factors */
static const float modeTorqueScale[3] = { 0.5f, 0.8f, 1.0f };   /* Eco, Normal, Sport */
static const float modeEnergyFactor[3] = { 0.7f, 1.0f, 1.4f };  /* higher = more SOC drain */
static const float modeRangeFactor[3]  = { 1.4f, 1.0f, 0.7f };  /* efficiency -> range     */

/* Raw 12-bit ADC snapshots filled by DMA/polling in the 10ms ISR */
static volatile uint16_t rawAccel, rawBrake, rawSoc, rawTemp;
static uint8_t socInitialized = 0;
static float   socFloat = 100.0f; /* internal float SOC accumulator for smooth decay */

void EV_Init(void)
{
    evData.mode      = MODE_NORMAL;
    evData.torqueNm  = 0.0f;
    evData.accelMps2 = 0.0f;
    evData.speedKmh  = 0.0f;
    evData.rangeKm   = 0.0f;
    socInitialized   = 0;
}

void EV_SetDriveMode(DriveMode_t mode)
{
    if (mode <= MODE_SPORT) {
        evData.mode = mode;
    }
}

/**
 * @brief Reads the 4 ADC channels (Accel, Brake, SOC, Temp) via polling.
 *        Called from inside the TIM1 10ms ISR.
 */
void EV_ReadPedalsAndSOC(void)
{
    uint32_t channels[4] = { ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3 };
    uint16_t results[4];

    for (int i = 0; i < 4; i++) {
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel      = channels[i];
        sConfig.Rank         = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
            results[i] = HAL_ADC_GetValue(&hadc1);
        } else {
            results[i] = 0;
        }
        HAL_ADC_Stop(&hadc1);
    }

    rawAccel = results[0];
    rawBrake = results[1];
    rawSoc   = results[2];
    rawTemp  = results[3];

    /* Scale raw 12-bit (0-4095) values to usable engineering units */
    evData.accelPct   = (uint8_t)((rawAccel * 100UL) / 4095UL);
    evData.brakePct   = (uint8_t)((rawBrake * 100UL) / 4095UL);
    evData.motorTempC = (uint16_t)((rawTemp * 150UL) / 4095UL);

    if (!socInitialized) {
        /* SOC potentiometer only sets the *initial* charge once at startup */
        socFloat = (float)((rawSoc * 100UL) / 4095UL);
        socInitialized = 1;
    }
    evData.soc = (uint8_t)socFloat;
}

/**
 * @brief Physics-based inertia model. Called once per 10ms tick after
 *        EV_ReadPedalsAndSOC(). dt_seconds should be 0.010f.
 */
void EV_ProcessStep(float dt_seconds)
{
    float scale = modeTorqueScale[evData.mode];

    if (evData.brakePct > BRAKE_THRESHOLD_PCT) {
        /* ---- Regenerative braking ---- */
        float regenTorque = -((float)evData.brakePct / 100.0f) * MAX_MOTOR_TORQUE_NM * REGEN_TORQUE_GAIN;
        evData.torqueNm = regenTorque;

        float decel = fabsf(regenTorque) / VEHICLE_MASS_KG; /* simplified F=ma -> a = T/m */
        evData.accelMps2 = -decel;

        float speedMps = evData.speedKmh / 3.6f;
        speedMps += evData.accelMps2 * dt_seconds;
        if (speedMps < 0.0f) speedMps = 0.0f;
        evData.speedKmh = speedMps * 3.6f;

        /* Recharge SOC proportionally to braking energy recovered */
        socFloat += ((float)evData.brakePct / 100.0f) * 0.01f;
        if (socFloat > 100.0f) socFloat = 100.0f;
    } else {
        /* ---- Normal driving / acceleration ---- */
        evData.torqueNm = ((float)evData.accelPct / 100.0f) * MAX_MOTOR_TORQUE_NM * scale;

        float driveForce = evData.torqueNm;
        float speedMps   = evData.speedKmh / 3.6f;
        float dragForce  = DRAG_COEFF * speedMps * speedMps;

        evData.accelMps2 = (driveForce - dragForce) / VEHICLE_MASS_KG;
        speedMps += evData.accelMps2 * dt_seconds;
        if (speedMps < 0.0f) speedMps = 0.0f;

        evData.speedKmh = speedMps * 3.6f;
        if (evData.speedKmh > MAX_SPEED_KMH) {
            evData.speedKmh = MAX_SPEED_KMH;
        }

        /* Battery drain proportional to torque demand & drive-mode energy factor, only while moving */
        if (evData.speedKmh > 0.0f) {
            float drain = (evData.torqueNm / MAX_MOTOR_TORQUE_NM) * modeEnergyFactor[evData.mode] * 0.002f;
            socFloat -= drain;
            if (socFloat < 0.0f) socFloat = 0.0f;
        }
    }

    evData.soc = (uint8_t)socFloat;

    /* Remaining range estimate based on current SOC and drive-mode efficiency */
    evData.rangeKm = socFloat * modeRangeFactor[evData.mode] * 1.5f;
}
