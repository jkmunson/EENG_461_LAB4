#include "main.h"
#include "gpioCode.h"
#include "adc.h"
#include "pwmCode.h"
#include "timers.h"
#include "uart_print.h"
#include "sw1_int.h"
#include <stdint.h>
#include <stdbool.h>
#include <common/tm4c123gh6pm.h>
#include <driverlib/rom.h>

int main (void) {

    //GPIO/Switch Configuration
	GPIOConfigure();
	configureDebounceTimer();

    //ADC Configuration
	ADCPinConfigure();
	ADCSampleSequencerConfigure();

    //PWM Configuration
	PWMConfigure();

    //printf UART setup
	setup_uart_printer();

	Enable_Interrupts(); //Enable Global Interrupts
	
	uint16_t last_distance = 0; //Last stored distance
	int32_t last_print_time = 0;
	
	while (1) {

        uint16_t distance_millimeters = potReading/2;

        set_angle = (distance_millimeters * DEG_OF_ROTATION) / 1000;

        /* If a duty cycle change occured, calculate new value and set pulse width */
        if (last_distance != distance_millimeters && (0 <= set_angle && set_angle <= DEG_OF_ROTATION)) {
            duty_cycle = (((set_angle * (MAX_PULSE - MIN_PULSE)/DEG_OF_ROTATION) + MIN_PULSE) / 200) * 100;
            PWMSetDutyCycle(duty_cycle);
            last_distance = distance_millimeters;
        }
		
		if(NEED_PRINT || (uptime_third_seconds > last_print_time)) {
			last_print_time = uptime_third_seconds;
			float voltage = ((float)potReading*3.3f)/4095.0f;
			printlf("The current ADC value is %d and the DC is %f \n\r", potReading, &voltage);
			printlf("System Clock: %d\n", ROM_SysCtlClockGet());
			NEED_PRINT = false;
		}
		
	}

	return (0);
}

/*
 * Taken from Lab Assignment
 */
void Disable_Interrupts(void) {
	__asm ("  CPSID	I\n");
}

void Enable_Interrupts(void) {
	__asm ("  CPSIE	I\n");
}
