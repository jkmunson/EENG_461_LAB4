#include "sw1_int.h"

#include <stdbool.h>
#include <stdint.h>

#include "common/tm4c123gh6pm.h"
#include "gpioCode.h"
#include "timers.h"
#include "uart_print.h"
#include <driverlib/rom.h>

#define MIN_CLOCKS_DEBOUNCE (ROM_SysCtlClockGet() >> 6)

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
	static uint64_t cycles_last = 0;
	static enum { RELEASED, PRESSED } button_state = RELEASED;
	
	const enum { FALLING, RISING } edge_type = (GPIO_PORTF_DATA_BITS_R[SW1_PIN] == SW1_PIN);
	
	//Early exit for don't-care state combinations
	if(	((button_state == PRESSED)  && (edge_type == FALLING )) ||
		((button_state == RELEASED) && (edge_type == RISING)) ) {
		return;
	}
	
	const uint64_t cycles_now = get_uptime_cycles();
	const uint64_t cycles_passed = cycles_now - cycles_last;
	if(cycles_passed > MIN_CLOCKS_DEBOUNCE) {
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
