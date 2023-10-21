#pragma once
#include <stdint.h>

#include "common/tm4c123gh6pm.h"

extern volatile uint32_t distance_millimeters;
extern volatile uint64_t sensor_trigger_start_time;

void configure_sonic_sensor(void);
void TIMER2B_INT_HANDELER(void);

#define SONIC_TRIG_PIN (1 << 0) //PB0 
#define SONIC_ECHO_PIN (1 << 1) //PB1
#define SONIC_PINS (SONIC_TRIG_PIN | SONIC_ECHO_PIN)