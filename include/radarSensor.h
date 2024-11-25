#include <Arduino.h>
#ifndef RADARSENSOR_H
#define RADARSENSOR_H

#define GATE_ACCUM    100   // number of intervals to accumulate
#define MULT_FACTOR   5     // multiply to get Hz output


extern volatile bool count_update;
extern volatile uint32_t count_output;

extern float Freq;

uint16_t read_radar_counter();
void QTIMER3_C2_setup(void);
//void PIT1_setup(void);
void TimerInit_Init(void);
void gate_timer(void);


#endif