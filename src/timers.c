#include "timers.h"
#include "main.h"
#include "common/tm4c123gh6pm.h"
#include "uart_print.h"

#define SECONDS_DIVISOR 6

volatile int32_t uptime_seconds;
volatile int32_t uptime_third_seconds;


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
    TIMER1_TAILR_R = CYCLES_PER_SEC/SECONDS_DIVISOR - 1;
    TIMER1_TAPR_R = 0;
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT; //Clear Interrupt
    TIMER1_IMR_R |= TIMER_IMR_TATOIM; //Enable Interrupt as Timeout
    NVIC_EN0_R = 1 << (INT_TIMER1A - 16);
    TIMER1_CTL_R |= TIMER_CTL_TAEN; //Enable Timer
}

void debounceTimerISR (void) {
	static char frac_second = 0;
	static char frac_third_second = 0;
    TIMER1_IMR_R &= ~TIMER_IMR_TATOIM; //Disable Interrupt
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT; //Clear Interrupt
	
	if (++frac_second > (SECONDS_DIVISOR-1)){
		uptime_seconds++;
		frac_second = 0;
	}
	
	if(++frac_third_second > (SECONDS_DIVISOR/3)-1){
		uptime_third_seconds++;
		frac_third_second = 0;
	}
	
	//Toggle trig pin
	
    TIMER1_IMR_R |= TIMER_IMR_TATOIM; //Enable Interrupt
}
