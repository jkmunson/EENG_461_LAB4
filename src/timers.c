#include "timers.h"
#include "main.h"
#include "common/tm4c123gh6pm.h"
#include "uart_print.h"
#include "sonic_sensor.h"
#include <stdbool.h>
#include <driverlib/rom.h>

#define TIMER1_MULTIPLIER 32 //Number of times timer1 will overflow, and trigger it's interrupt each second
#define TIMER_CYCLES 500000//(ROM_SysCtlClockGet()/TIMER1_MULTIPLIER)

volatile int32_t uptime_seconds;
volatile uint64_t timer1_overflow_count;

#define TIMER_ISR_IS_PENDING (TIMER1_MIS_R & TIMER_ICR_TATOCINT)

void configureAdcTimer (void) {

	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0; //Enable Run Mode Clock Gating Control for Timer 0

	while (!(SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R0)) {}

	TIMER0_CTL_R &= ~TIMER_CTL_TAEN; //Disable Timer
	TIMER0_CTL_R |= TIMER_CTL_TASTALL; //Stall for debug
	TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;
	TIMER0_TAMR_R |= TIMER_TAMR_TAMR_PERIOD; //Set Timer to count down periodically
	TIMER0_TAILR_R = 16000 - 1;
	TIMER0_CTL_R |= TIMER_CTL_TAOTE; //Set as an ADC Trigger
	TIMER0_CTL_R |= TIMER_CTL_TAEN; //Enable Timer
}

void configureDebounceTimer(void) {

	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; //Enable Run Mode Clock Gating Control for Timer 0

	while (!(SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R1)) {}

	TIMER1_CTL_R &= ~TIMER_CTL_TAEN; //Disable Timer
	TIMER1_CTL_R |= TIMER_CTL_TASTALL; //Stall for debug
	TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
	TIMER1_TAMR_R |= TIMER_TAMR_TAMR_PERIOD; //Set Timer to count down periodically
	TIMER1_TAILR_R = TIMER_CYCLES-1;
	TIMER1_TAPR_R = 0;
	TIMER1_ICR_R |= TIMER_ICR_TATOCINT; //Clear Interrupt
	TIMER1_IMR_R |= TIMER_IMR_TATOIM; //Enable Interrupt as Timeout
	NVIC_EN0_R = 1 << (INT_TIMER1A - 16);
	TIMER1_CTL_R |= TIMER_CTL_TAEN; //Enable Timer
}



void timeKeeperISR (void) {
	static char second_counter = 0;
	static char sonic_sensor_action_div = 0;
	
	TIMER1_IMR_R &= ~TIMER_IMR_TATOIM; //Disable Interrupt
	TIMER1_ICR_R |= TIMER_ICR_TATOCINT; //Clear Interrupt
	
	timer1_overflow_count++;
	
	//Every second
	if(++second_counter == TIMER1_MULTIPLIER) {
		uptime_seconds++;
		second_counter = 0;
	}
	
	if(++sonic_sensor_action_div == TIMER1_MULTIPLIER/16) {
		// Start pulse with sonic sensor.
		GPIO_PORTB_DATA_BITS_R[SONIC_TRIG_PIN] = SONIC_TRIG_PIN;
		sensor_trigger_start_time = get_uptime_cycles();
		sonic_sensor_action_div = 0;
	}
	
	TIMER1_IMR_R |= TIMER_IMR_TATOIM; //Enable Interrupt
}

uint64_t get_uptime_cycles(void) {
	uint64_t overflow_count_now;
	uint64_t cycles_now;
	
	do {
		if(TIMER_ISR_IS_PENDING) timeKeeperISR();
		overflow_count_now = timer1_overflow_count;
		cycles_now = TIMER_CYCLES - TIMER1_TAR_R;
	// If the counter overflowed during this code block, then our reads of uptime and cycles are invalid. Re-do them.
	} while (TIMER_ISR_IS_PENDING); 
	
	return (TIMER_CYCLES * overflow_count_now) + cycles_now;
}











