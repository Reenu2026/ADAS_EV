#ifndef __FSM_FAULT_H
#define __FSM_FAULT_H

#include "main.h"

typedef enum {
    STATE_PARKED = 0,
    STATE_READY,
    STATE_DRIVING,
    STATE_REGEN,
    STATE_FAULT
} VehicleState_t;

/* Fault flags (bitmask) */
#define FAULT_NONE           0x00
#define FAULT_OVERHEAT       0x01
#define FAULT_LOW_BATTERY    0x02
#define FAULT_COLLISION      0x04

extern VehicleState_t vehicleState;
extern uint8_t         faultFlags;

void FSM_Init(void);
/* Call every 10ms (or every cycle) after EV + ADAS data is refreshed */
void FSM_Update(void);
void FSM_ClearFault(void);

#endif /* __FSM_FAULT_H */
