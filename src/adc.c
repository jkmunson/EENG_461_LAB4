#include <stdint.h>
#include "adc.h"
#include "timers.h"
#include "common/tm4c123gh6pm.h"

#define GPIO_PIN4 (1 << 4)
#define POT_TRIGGER_MARGIN 0xA

volatile uint16_t potReading;

void ADCPinConfigure(void) {

    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;                      //Enable ADC Clock
    while(!(SYSCTL_PRADC_R & SYSCTL_PRADC_R0)) {};              //Wait for peripheral to be ready

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;                    //Enable GPIO Pin for ADC (PE5)
    while(!(SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R4)) {};        //Wait fo peripheral to be ready

    GPIO_PORTE_AFSEL_R |= GPIO_PIN4;                            //Set Alternate Function Select
    GPIO_PORTE_DEN_R &= ~GPIO_PIN4;                             //Clear Digital Enable for Pin 5
    GPIO_PORTE_AMSEL_R |= GPIO_PIN4;                            //Set Alternate Mode Select

}

void ADCSampleSequencerConfigure(void) {

    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                           //Disable Sequencer 3
    ADC0_EMUX_R |= ADC_EMUX_EM3_TIMER;                          //Set ADC as Timer Triggered
    ADC0_SSMUX3_R |= 0x9;                                       //Enable AIN9
    ADC0_SSCTL3_R |= ADC_SSCTL3_IE0 | ADC_SSCTL3_END0;          //Sequencer control
    ADC0_SAC_R = 0x6;                                           //Enables x64 Oversampling
    ADC0_ISC_R |= ADC_ISC_IN3;                                  //Clear Interrupt
    ADC0_IM_R |= ADC_IM_MASK3;                                  //Enable Interrupt
    NVIC_EN0_R |= 1 << (INT_ADC0SS3 - 16);                      //Enable NVIC for ADC0 Sequencer 3

    configureAdcTimer();

    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                            //Enable Sequencer

}

void saveADCSample(void){

    ADC0_IM_R &= ~ADC_IM_MASK3;                                 //Disable Interrupt
    ADC0_ISC_R |= ADC_ISC_IN3;                                  //Clear Interrupt

    potReading = (ADC0_SSFIFO3_R & ADC_SSFIFO3_DATA_M);         //Read Potentiometer Value

    ADC0_IM_R |= ADC_IM_MASK3;                                  //Enable Interrupt

}
