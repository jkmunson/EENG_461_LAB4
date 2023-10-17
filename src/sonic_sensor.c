#include <sonic_sensor.h>

#include "common/tm4c123gh6pm.h"

#define SONIC_TRIG_PIN	// 
#define SONIC_ECHO_PIN (1 << 1) //PB1

void configure_sonic_sensor(void){
	
	//set up GPIOs
	
	//set up timer2 as input capture
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2; //Enable Run Mode Clock Gating Control for Timer 0
    while (!(SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R1)) {}
    

	
	
}

