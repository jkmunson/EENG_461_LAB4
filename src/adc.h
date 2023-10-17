#ifndef EENG461_LAB_3_ADC_H
#define EENG461_LAB_3_ADC_H

#include <stdint.h>

extern volatile uint16_t potReading;

void ADCPinConfigure(void);
void ADCSampleSequencerConfigure(void);
void saveADCSample(void);

#endif //EENG461_LAB_3_ADC_H
