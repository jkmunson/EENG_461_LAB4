#include "sw1_int.h"

#include <stdbool.h>
#include <stdint.h>

#include "common/tm4c123gh6pm.h"
#include "gpioCode.h"
#include "timers.h"
#include "uart_print.h"

#define MIN_CLOCKS_DEBOUNCE (int32_t)((int32_t)CYCLES_PER_SEC/100)

#define TIMER_ISR_IS_PENDING (TIMER1_MIS_R & TIMER_ICR_TATOCINT)

bool NEED_PRINT = false;

static void sw1_debounce(void);
static void sw1_action(void);

void PORTF_int_handler(void){
	// ASSUMPTION: ONLY SW1 ON PORTF is toggled. 
	// If more pins were being toggled, then we would need to dispatch a different handeler for each.
	// Such code might look like this.
	
	if(GPIO_PORTF_MIS_R & SW1_PIN) {
		GPIO_PORTF_IM_R &= ~SW1_PIN; //Disable interrupt
		GPIO_PORTF_ICR_R |= SW1_PIN; //Clear interrupt
		sw1_debounce();
		GPIO_PORTF_IM_R |= SW1_PIN; //Re-enable interrupt
	}
}

//Performs debounce by disallowing further input after the first edge transition until MIN_CLOCKS_DEBOUNCE has passed.
static void sw1_debounce(void){
	static int32_t uptime_last = 0; 
	static int32_t cycles_last = 0;
	static enum { RELEASED, PRESSED } button_state = RELEASED;
	
	const enum { FALLING, RISING } edge_type = (GPIO_PORTF_DATA_BITS_R[SW1_PIN] == SW1_PIN);
	
	int32_t uptime_now;
	int32_t cycles_now;
	
	// Early exit for don't-care state combinations
	if(	((button_state == PRESSED)  && (edge_type == FALLING )) ||
		((button_state == RELEASED) && (edge_type == RISING)) ) {
		return;
	}
	
	//Critical Section: read of the current time
	__asm("CPSID I\n"); //Disable interrupt handling
	do {
		if(TIMER_ISR_IS_PENDING) debounceTimerISR();
		
		uptime_now = uptime_seconds;
		cycles_now = CYCLES_PER_SEC-(TIMER1_TAR_R*3);
		
	// If the counter overflowed during this code block, then our reads of uptime and cycles are invalid. Re-do them.
	} while (TIMER_ISR_IS_PENDING); 
	__asm("CPSIE I\n"); //Re-enable interrupt handeling
	
	int32_t seconds_passed = (uptime_now - uptime_last);
	if(seconds_passed > 2) seconds_passed = 2; // Prevent overflow
	int32_t cycles_passed  = (cycles_now - cycles_last) + seconds_passed*CYCLES_PER_SEC;
	if(cycles_passed > MIN_CLOCKS_DEBOUNCE) {
		uptime_last = uptime_now;
		cycles_last = cycles_now;
		
		switch(button_state) {
			case PRESSED:
				button_state = RELEASED;
			break;
			
			case RELEASED:
				button_state = PRESSED;
				sw1_action();
		}
	} 
}

static void sw1_action(void) {
	NEED_PRINT = true;
}
