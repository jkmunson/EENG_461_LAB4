#include <stdint.h>
#include "pwmCode.h"
#include "common/tm4c123gh6pm.h"

void PWMConfigure(void) {
    /*
     * Red Led (PF1) is on M1PWM5G2
     */

    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;                                  //Enable PWM Module 1
    while(!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R1)) {}                           //Wait for peripheral to be ready

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                                //Enable GPIO Port F
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)) {}                         //Wait for peripheral to be ready

    GPIO_PORTF_AFSEL_R |= (1 << 1);                                         //Set Alternate Function for PF1
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5;                              //Set Port Control to PF1 PWM value
    GPIO_PORTF_DEN_R |= (1 << 1);                                           //Set Digital Enable for PF1

    PWM1_2_CTL_R = 0x0;                                                     //Disable PWM1 Gen2
    PWM1_2_GENB_R = PWM_2_GENB_ACTCMPBD_ONE | PWM_2_GENB_ACTLOAD_ZERO | PWM_2_GENB_ACTZERO_ONE;
                                                                            //Set PWM1 Gen2-B comparator, load, and zero

    PWM1_2_LOAD_R = CYCLES_PER_MS;                                          //Set PWM Gen2 Period to 1ms
    PWM1_2_CMPB_R = 0x0;                                                    //Set Comparator value
    PWMEnable();                                                            //Enable PWM

}

void PWMSetPeriod(uint16_t cycles_per_period) {
    PWMDisable();                                                           //Disable PWM
    PWM1_2_LOAD_R = cycles_per_period;                                      //Set new period
    PWMEnable();                                                            //Enable PWM
}

void PWMSetDutyCycle(uint8_t duty_cycle) {
    PWMDisable();                                                           //Disable PWM
    PWM1_2_CMPB_R = CYCLES_PER_MS_DIV_100 * (duty_cycle);                   //Set new duty cycle
    PWMEnable();                                                            //Enable PWM
}

void PWMEnable(void) {
    PWM1_2_CTL_R |= PWM_2_CTL_ENABLE;                                       //Enable PWM1 Gen 2
    PWM1_ENABLE_R |= PWM_ENABLE_PWM5EN;                                     //Enable PWM1 Output 5
}

void PWMDisable(void) {
    PWM1_2_CTL_R &= ~PWM_2_CTL_ENABLE;                                      //Disable PWM1 Gen 2
    PWM1_ENABLE_R &= ~PWM_ENABLE_PWM5EN;                                    //Disable PWM1 Output 5
}
