#include "main.h"
#include "gpioCode.h"
#include "adc.h"
#include "pwmCode.h"
#include "timers.h"
#include "uart_print.h"
#include "sonic_sensor.h"
#include "sw1_int.h"
#include <stdint.h>
#include <stdbool.h>
#include <common/tm4c123gh6pm.h>
#include <driverlib/rom.h>

int main (void) {
	//printf UART setup
	setup_uart_printer();
	configure_sonic_sensor();
	
    //GPIO/Switch Configuration
	GPIOConfigure();
	configureDebounceTimer();

    //ADC Configuration
	ADCPinConfigure();
	ADCSampleSequencerConfigure();

    //PWM Configuration
	PWMConfigure();

	Enable_Interrupts(); //Enable Global Interrupts
	
	uint16_t last_distance = 0; //Last stored distance
	int32_t last_print_time = 0;
	
	while (1) {

        uint16_t distance_millimeters = potReading/2;

        set_angle = (distance_millimeters * DEG_OF_ROTATION) / 1000;
		
		printlf("Reg: %u\n", TIMER2_TBV_R);
	}{
		
        /* If a duty cycle change occured, calculate new value and set pulse width */
        if (last_distance != distance_millimeters && (0 <= set_angle && set_angle <= DEG_OF_ROTATION)) {
            duty_cycle = (((set_angle * (MAX_PULSE - MIN_PULSE)/DEG_OF_ROTATION) + MIN_PULSE) / 20);
            PWMSetDutyCycle(duty_cycle);
            last_distance = distance_millimeters;
        }
		
		if(NEED_PRINT || (uptime_seconds-last_print_time)) {
			last_print_time = uptime_seconds;
			printlf("[%d] The current distance_millimeters value is %d \n\r", uptime_seconds, distance_millimeters);
			NEED_PRINT = false;
		}
		
		//NOTE for Dr Meyer: There is not a specific requirement for the length of the trigger pulse, it simple needs to be over 10us
		//And the ping transmits after the falling edge.
		//End the output pulse for the sonic trigger. Guarenteed to exit early if the pin is already off without calling get_uptime_cycles()
		if(GPIO_PORTB_DATA_BITS_R[SONIC_TRIG_PIN] && ((get_uptime_cycles() - sensor_trigger_start_time) > (ROM_SysCtlClockGet()>>13) ) ) {
			GPIO_PORTB_DATA_BITS_R[SONIC_TRIG_PIN] = 0;
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
