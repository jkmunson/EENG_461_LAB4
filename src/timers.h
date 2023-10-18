#pragma once
#include "stdint.h"

extern volatile int32_t uptime_seconds;

void configureAdcTimer (void);
void configureDebounceTimer(void);
void ADCTrigger(void);
uint64_t get_uptime_cycles(void);
void timeKeeperISR (void);
