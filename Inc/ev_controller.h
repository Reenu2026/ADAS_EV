#ifndef __EV_CONTROLLER_H
#define __EV_CONTROLLER_H

#include "main.h"

typedef enum {
    MODE_ECO = 0,
    MODE_NORMAL,
    MODE_SPORT
} DriveMode_t;

typedef struct {
    /* Raw scaled inputs */
    uint8_t  accelPct;      /* 0-100 %   */
    uint8_t  brakePct;      /* 0-100 %   */
    uint8_t  soc;           /* 0-100 %   */
    uint16_t motorTempC;    /* 0-150 C   */

    /* Derived EV metrics */
    float    torqueNm;      /* Motor torque (Nm), can be negative on regen */
    float    accelMps2;     /* Vehicle acceleration (m/s^2) */
    float    speedKmh;      /* Vehicle speed (km/h) */
    float    rangeKm;       /* Remaining range (km) */
    DriveMode_t mode;
} EVData_t;

extern EVData_t evData;

void EV_Init(void);
void EV_ReadPedalsAndSOC(void);          /* Called from TIM1 10ms ISR context (sets flag) */
void EV_ProcessStep(float dt_seconds);   /* Physics/inertia model update, dt = 0.010s      */
void EV_SetDriveMode(DriveMode_t mode);

#endif /* __EV_CONTROLLER_H */
