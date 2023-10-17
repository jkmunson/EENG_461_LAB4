#include "common/tm4c123gh6pm.h"
#include "gpioCode.h"
#include <stdint.h>

void GPIOConfigure(void) {
	// Enable GPIO clock
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
	while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)) {};
	
	// Configure pins
	GPIO_PORTF_DEN_R |= RGB_PINS;
	GPIO_PORTF_DR8R_R |= RGB_PINS;
	
	// Set initial values
	GPIO_PORTF_DATA_R &= ~RGB_PINS; //All off to start
	
	// Set pin directions
	GPIO_PORTF_DIR_R |= RGB_PINS;
	
	//SW1 pullup
	GPIO_PORTF_PUR_R |= SW1_PIN;
	GPIO_PORTF_DEN_R |= SW1_PIN;
	
	//Enable interrupts on value of buttons
	GPIO_PORTF_IS_R &= ~SW1_PIN; //Edge triggered
	GPIO_PORTF_IBE_R |= SW1_PIN; //Both Edges
	GPIO_PORTF_IM_R |= SW1_PIN;  //Unmask the pin
	
	//NVIC_PRI7_R = (NVIC_PRI7_R & ~NVIC_PRI7_INT30_M) | (0x7 << NVIC_PRI7_INT30_S); //Set the PORTF interrupt to the lowest priority.
	
	NVIC_EN0_R |= (1 << 30); 	 // Enable Port F interrupts in nvic
}
