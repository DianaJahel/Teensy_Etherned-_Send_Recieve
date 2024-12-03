#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include "dataTransfer.h"
#include "commandsDef.h"
#include "commandRoutines.h"
#include "adcPart.h"
#include "WireSensor.h"
#include "radarSensor.h"

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
 
}

void loop() {
 

}
