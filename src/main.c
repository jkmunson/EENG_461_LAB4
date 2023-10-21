#include <stdint.h>
#include <stdbool.h>

#include <common/tm4c123gh6pm.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>

#include "main.h"
#include "gpioCode.h"
#include "adc.h"
#include "pwmCode.h"
#include "timers.h"
#include "uart_print.h"
#include "sonic_sensor.h"
#include "sw1_int.h"

#define Disable_Interrupts() __asm("CPSID I")
#define Enable_Interrupts() __asm("CPSIE I")


int main (void) {
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_12_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
	setup_uart_printer();
	
	GPIOConfigure();
	configureDebounceTimer();

	//ADCPinConfigure();
	//ADCSampleSequencerConfigure();

	PWMConfigure();
	
	configure_sonic_sensor();

	Enable_Interrupts(); //Enable Global Interrupts
	
	uint16_t last_distance = 0; //Last stored distance
	int32_t last_print_time = 0;
	
	while (1) {
		/* If a duty cycle change occured, calculate new value and set pulse width */
		if (last_distance != distance_millimeters) {
			set_angle = (distance_millimeters * DEG_OF_ROTATION) / 1000;
			if(set_angle > DEG_OF_ROTATION) set_angle = DEG_OF_ROTATION;
			
			duty_cycle = (((set_angle * (MAX_PULSE - MIN_PULSE)/DEG_OF_ROTATION) + MIN_PULSE) / 20);
			PWMSetDutyCycle(duty_cycle);
			last_distance = distance_millimeters;
		}
		
		if(NEED_PRINT || (uptime_seconds-last_print_time)) {
			last_print_time = uptime_seconds;
			printlf("[%u] Distance: %u \t \n\r", uptime_seconds, distance_millimeters);
			NEED_PRINT = false;
		}
		
		//NOTE for Dr Meyer: There is not a specific requirement for the length of the trigger pulse, it simply needs
		//to be over 10us then the ping transmits after the falling edge.
		
		//End the output pulse for the sonic trigger it it's been over 10us
		if(GPIO_PORTB_DATA_BITS_R[SONIC_TRIG_PIN] && ((get_uptime_cycles() - sensor_trigger_start_time) > (ROM_SysCtlClockGet()>>13) ) ) {
			GPIO_PORTB_DATA_BITS_R[SONIC_TRIG_PIN] = 0;
		}
	}

	return (0);
}