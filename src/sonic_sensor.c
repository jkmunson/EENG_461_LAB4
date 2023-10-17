#include <sonic_sensor.h>

#include "common/tm4c123gh6pm.h"

#define SONIC_TRIG_PIN	// 
#define SONIC_ECHO_PIN (1 << 1) //PB1

void configure_sonic_sensor(void){
	
	//set up GPIOs
	
	//set up timer2 as input capture
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2; //Enable Run Mode Clock Gating Control for Timer 0
    while (!(SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R1)) {}
    /*
	TIMER2_CTL_R &= ~TIMER_CTL_TAEN; //Disable Timer
    TIMER2_CTL_R &= ~TIMER_CTL_TASTALL; //Stall for debug
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR | TIMER_TAMR_TAPWMIE; //Capture Mode | Capture Time | Count Up | Interrupt on capture events
    TIMER2_TAILR_R = CYCLES_PER_SEC/SECONDS_DIVISOR - 1;
    TIMER2_TAPR_R = 0;
    TIMER2_ICR_R |= TIMER_ICR_TATOCINT; //Clear Interrupt
    TIMER2_IMR_R |= TIMER_IMR_TATOIM; //Enable Interrupt as Timeout
    NVIC_EN0_R = 1 << (INT_TIMER1A - 16);
    TIMER1_CTL_R |= TIMER_CTL_TAEN; //Enable Timer*/
	
	
}

