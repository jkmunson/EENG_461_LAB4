#include <sonic_sensor.h>

#include "common/tm4c123gh6pm.h"

volatile uint16_t distance_millimeters;
volatile uint64_t sensor_trigger_start_time;

void configure_sonic_sensor(void){

	//set up GPIOs
	// Enable GPIO clock
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
	while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1)) {};
	
	GPIO_PORTB_DIR_R |= SONIC_TRIG_PIN; //Set to output
	
	GPIO_PORTB_DEN_R |= SONIC_PINS; //Enable pins
	
	//T2CCP1
	
	
	//set up timer2 as input capture
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2; //Enable Run Mode Clock Gating Control for Timer 0
	while (!(SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R1)) {}
	
	TIMER2_CTL_R &= ~TIMER_CTL_TAEN; //Disable Timer
	
	TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER; // 32 bit mode
	
	TIMER2_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR | TIMER_TAMR_TAPWMIE; //Capture Mode | Capture Time | Count Up | Interrupt on capture events
	
	TIMER2_CFG_R |= TIMER_CTL_TAEVENT_BOTH | TIMER_CTL_TASTALL; //Capture events on both edges, Stop timer during debug
	
	TIMER2_IMR_R |= TIMER_IMR_CAEIM; // Enable interrupts on capture events

	TIMER1_CTL_R |= TIMER_CTL_TAEN; //Enable Timer
	
	
}

