#ifndef ADCPART_H
#define ADCPART_H
#include <Arduino.h>



void init_adc1();
void ADC1_IRQHandler();
void start_adc_conversion();
void createDataPacket();

#endif //ADCPART_H

