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
	
	uint16_t duty_cycle_last = 0; //Last stored/set duty cycle
	int32_t last_print_time = 0;
	
	while (1) {

        //Calculate a corresponding duty cycle percentage
		uint16_t temp_duty_cycle = (uint16_t)((potReading*100)/4095);

        /*
         * Only set a new duty cycle if the potentiometer value changed
         */
		if (temp_duty_cycle != duty_cycle_last) {
			PWMSetDutyCycle(temp_duty_cycle);
			duty_cycle_last = temp_duty_cycle;
		}
		
		if(NEED_PRINT || (uptime_third_seconds > last_print_time)) {
			last_print_time = uptime_third_seconds;
			float voltage = ((float)potReading*3.3f)/4095.0f;
			printlf("The current ADC value is %d and the DC is %f \n\r", potReading, &voltage);
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
