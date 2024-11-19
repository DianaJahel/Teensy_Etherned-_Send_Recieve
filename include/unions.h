#include <Arduino.h>
#ifndef UNIONS_H
#define UNIONS_H
union floatData {
  float value;
  byte array[4];
};

union uint16Data{
  uint16_t value;
  byte array[2];
};

union {
  int16_t value;
  byte array[2];
} int16Data;

union uint32Data{
  uint32_t value;
  byte array[4];
};

union int32Data{
  int32_t value;
  byte array[4];
};

#endif //UNIONS_H
