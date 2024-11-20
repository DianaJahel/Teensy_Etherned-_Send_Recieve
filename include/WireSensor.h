#include <Arduino.h>

#ifndef WIRESENSOR_H
#define WIRESENSOR_H

extern float calculo;
extern float  wire_dist;

float read_wire_sensor();
void QTIMER3_C1_C0_setup();

#endif