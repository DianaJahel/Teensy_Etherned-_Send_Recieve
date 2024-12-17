
/*
This code implements multiple sensors and inputs on Teensy4.1 board
 1 Radar sensor on pin 14
 1 Wrie position sensor on pins 18,19
 3 ADC inputs on pins 20, 22, 23 

*/

#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include "dataTransfer.h"
#include "commandsDef.h"
#include "commandRoutines.h"
#include "adcPart.h"
#include "WireSensor.h"
#include "radarSensor.h"
#include "watchDog1.h"

#define GATE_INTERVAL_DATA 10000  // microseconds for each gate interval
#define GATE_ACCUM_DATA    100   // number of intervals to accumulate


void periodic_data_transfer (){

  if(!distance_transfer_mode){
    decodeUdpCommands();
    decodeControlCommands();
    dataTransfer();
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  interrupts();
  //sei();
 
  initEthernet();
  init_adc1();
  
  pinMode(LED_BUILTIN, OUTPUT);
  QTIMER3_C1_C0_setup(); //Init wire sensor
  QTIMER3_C2_setup();  //Init Radar sensor
  
  analogWriteFrequency(2, 1200);
  analogWrite(2, 128);

  GPT1_Init(); //Data transfer based on distance (Radar sensor)
	pit_timer_sendData.begin(periodic_data_transfer, GATE_INTERVAL_DATA); //Fixed frequency data transfer PIT 1
  
  WDOG_init(); //6.5 seconds
 
}

void loop() {
 
  
   WDOG1_WSR = 0x5555;  //Watchdog Service
   WDOG1_WSR = 0xAAAA;

}
