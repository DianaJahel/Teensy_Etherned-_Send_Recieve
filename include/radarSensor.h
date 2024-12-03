#include <Arduino.h>
#ifndef RADARSENSOR_H
#define RADARSENSOR_H

#define GATE_ACCUM    100   // number of intervals to accumulate
#define MULT_FACTOR   5     // multiply to get Hz output


extern volatile bool count_update;
extern volatile uint32_t count_output;

extern float Freq;

extern unsigned int data_pulses;

extern bool distance_transfer_mode;

//extern unsigned int pulses_per_meter;
extern double total_dist;
uint16_t read_distance_counter();
uint16_t read_radar_counter();
void QTIMER3_C2_setup(void);

void GPT1_Init();
void gate_timer(void);


#endif