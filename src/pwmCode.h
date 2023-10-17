#include <stdint.h>
#ifndef EENG461_LAB_3_PWMCODE_H
#define EENG461_LAB_3_PWMCODE_H

#define RED_LED (1 << 1)
#define CYCLES_PER_MS 15999
#define CYCLES_PER_MS_DIV_100 159
#define DC_TOLERANCE 1

void PWMConfigure(void);
void PWMSetPeriod(uint16_t cycles_per_period);
void PWMSetDutyCycle(uint8_t duty_cycle);
void PWMEnable(void);
void PWMDisable(void);

#endif //EENG461_LAB_3_PWMCODE_H
