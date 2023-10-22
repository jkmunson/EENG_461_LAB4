#include <sonic_sensor.h>

#include "common/tm4c123gh6pm.h"
#include "driverlib/rom.h"
#include <stdint.h>
#include <stdbool.h>
#include "uart_print.h"

volatile uint32_t distance_millimeters;
volatile uint64_t sensor_trigger_start_time;

#define CAPTURE_TIMER_PRESCALE 128

void configure_sonic_sensor(void){
	//Enable peripheral and wait for ready
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
	while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1)) {};
	
	GPIO_PORTB_DIR_R |= SONIC_TRIG_PIN; //Set to output
	
	GPIO_PORTB_AFSEL_R |= SONIC_ECHO_PIN; //Enable alternate function
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & (~GPIO_PCTL_PB1_M)) | GPIO_PCTL_PB1_T2CCP1; //Timer2 capture 1
	
	GPIO_PORTB_DEN_R |= SONIC_PINS; //Enable pins

	//Timer2 as input capture
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2; //Enable Run Mode Clock Gating Control for Timer 2
	while (!(SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R2)) {}
	
	TIMER2_CTL_R &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN); //Disable Timer
	
	TIMER2_CFG_R = (TIMER2_CFG_R & (~TIMER_CFG_M)) | TIMER_CFG_16_BIT; // 16 bit mode - must be split for capture
	TIMER2_TBMR_R = TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR; // Capture Mode, count up, edge-time mode, 
	TIMER2_CTL_R |= TIMER_CTL_TBEVENT_BOTH; 	// Capture both edges
	TIMER2_IMR_R |= TIMER_IMR_CBEIM; 			// Unmask capture event interrupt
	TIMER2_ICR_R = TIMER_ICR_CBECINT; 			// Clear capture interrupt flag
	NVIC_EN0_R |= 1 << (INT_TIMER2B - 16);		// Enable interrupt in NVIC
	TIMER2_TBPR_R = (TIMER2_TBPR_R & (~TIMER_TBPR_TBPSR_M)) | CAPTURE_TIMER_PRESCALE;
	TIMER2_CTL_R |= TIMER_CTL_TBEN; 			// Enable Timer

}

void TIMER2B_INT_HANDELER(void){
	TIMER2_ICR_R = TIMER_ICR_CBECINT; // Ack interrupt
	static uint32_t cycles_per_mm;
	static uint32_t cycles_rise = 0;
	
	// The division and function call are potentially expensive. Doing this to avoid hard-coding the system clock
	// but still minimizing the run-time impact.
	static bool cycles_per_mm_is_initialized = false;
	if(!cycles_per_mm_is_initialized) {
		cycles_per_mm = (ROM_SysCtlClockGet() << 1) / 340270;
		cycles_per_mm_is_initialized = true;
	}
	
	const enum {RISING, FALLING} edge_type = (GPIO_PORTB_DATA_BITS_R[SONIC_ECHO_PIN] == SONIC_ECHO_PIN) ? RISING : FALLING;
	
	switch (edge_type){
		case RISING: {
			cycles_rise = TIMER2_TBR_R;
		} return;
		
		case FALLING: {
			const uint32_t cycles_fall = TIMER2_TBR_R;
			
			//If no time wrap has occurred, then it's the difference. Otherwise, it's the max value minus the difference
			//This assumes that it hasn't been more than twice the timer period. 
			const uint32_t cycles_passed = (cycles_fall > cycles_rise) ? (cycles_fall - cycles_rise) : (0xFFFFFF) - (cycles_rise - cycles_fall);
			
			const uint32_t temp_distance = cycles_passed / cycles_per_mm;
			if (temp_distance > 2000) return; //Ignore values too large - likely a lost/reflected/timeout pulse
			
			distance_millimeters = ((distance_millimeters*7) + temp_distance) / 8; //Average in values over 8 valid readings
		} return;
	}
}
