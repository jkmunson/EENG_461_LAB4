#include <stdint.h>
#ifndef EENG461_LAB_3_PWMCODE_H
#define EENG461_LAB_3_PWMCODE_H

#define RED_LED (1 << 1)
#define CYCLES_PER_MS 1000
#define DC_TOLERANCE 1

//Servo Globals
#define SERVO_PERIOD 0.02     // 20ms period
#define MEDIAN_PULSE 1.5      // 1.5ms pulse
#define MIN_PULSE 0.5f         // -90deg pulse (~0.5ms pulse)
#define MAX_PULSE 2.5f         // 90deg pulse (~2.5ms pulse)
#define DEG_OF_ROTATION 180.0f // How many degrees the servo can rotate
extern volatile float set_angle;            // 0 <= set_angle <= 180
extern volatile float duty_cycle;       // Percent in decimal form

void PWMConfigure(void);
void PWMSetPeriod(uint16_t cycles_per_period);
void PWMSetDutyCycle(float duty_cycle);
void PWMEnable(void);
void PWMDisable(void);

#endif //EENG461_LAB_3_PWMCODE_H
