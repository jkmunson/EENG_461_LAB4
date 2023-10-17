#pragma once
#include "stdint.h"

extern volatile int32_t uptime_seconds;
extern volatile int32_t uptime_third_seconds;

#define CYCLES_PER_SEC 16000000

void configureAdcTimer (void);
void configureDebounceTimer(void);
void debounceTimerISR (void);
void ADCTrigger(void);
