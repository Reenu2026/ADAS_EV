#ifndef __ADAS_SYSTEM_H
#define __ADAS_SYSTEM_H

#include "main.h"

typedef struct {
    float frontDistanceCm;
    float leftDistanceCm;
    float rightDistanceCm;

    float ttcSeconds;       /* Time-to-collision, default 99.9s */

    uint8_t criticalAlarm;  /* Front <20cm OR TTC<1.5s               */
    uint8_t warningAlarm;   /* Front <50cm OR TTC<3.0s                */
    uint8_t blindSpotLeft;  /* Speed>20 && Left <30cm                 */
    uint8_t blindSpotRight; /* Speed>20 && Right <30cm                */
} ADASData_t;

extern ADASData_t adasData;

void ADAS_Init(void);
void ADAS_ScanSensors(float currentSpeedKmh); /* Called every 100ms from TIM3 ISR context */

#endif /* __ADAS_SYSTEM_H */
