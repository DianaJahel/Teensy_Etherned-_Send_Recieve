//Since the Watchdog functionality of the radar sensor is made dy the PIT1, part of the speed calcuation is done on the void PIT_IRQHandler(void) 
//can be found on periodicTimer.cpp

#include <Arduino.h>
#include "radarSensor.h"
#include "dataTransfer.h"

//Pulses per meter 100 pulses/meter

volatile bool count_update = false;
volatile uint32_t count_output=0;


long curr_dist = 0;
bool distance_transfer_mode = false;

unsigned int data_pulses= 10;
#define pulses_per_meter 10.0
double total_dist= 0.0;

#define GATE_INTERVAL_RADAR 2000  // microseconds for each gate interval
#define GATE_ACCUM_RADAR    100   // number of intervals to accumulate
#define MULT_FACTOR   5     // multiply to get Hz output

float Freq =0;
unsigned long mycounter= 0;



typedef struct {
	IMXRT_TMR_t *timer;
	int timerchannel;
	int pin;
	int pinconfig;
	volatile uint32_t *inputselectreg;
	int inputselectval;
} timerinfo_t;

const timerinfo_t Radar_counter = {&IMXRT_TMR3, 2,  14,  1, &IOMUXC_QTIMER3_TIMER2_SELECT_INPUT, 1};

const timerinfo_t distance_counter = {&IMXRT_TMR3, 3,  15,  1, &IOMUXC_QTIMER3_TIMER3_SELECT_INPUT, 1};

uint16_t read_distance_counter() {
  uint16_t mcount = distance_counter.timer->CH[distance_counter.timerchannel].CNTR;
  return mcount;
}
uint16_t read_radar_counter() {
	static uint16_t prior = 0;
	uint16_t count = Radar_counter.timer->CH[Radar_counter.timerchannel].CNTR;

	uint16_t inc = count - prior;
	prior = count;
	return inc;

	//return count;
}


void gate_timer() {
    static unsigned int count = 0;
    static uint32_t accum = 0;

    accum += read_radar_counter();
    if (++count >= GATE_ACCUM) {
      count_output = accum;
      accum = 0;
      count_update = true;
      count = 0;
    }
}

void QTIMER3_C2_setup(void){  //Frequencimeter Radar sensor
  // turn on clock to the specific quad timer (QuadTimer3)
	CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
  CCM_CCGR4 |= CCM_CCGR4_IOMUXC(CCM_CCGR_ON);
  TMR3_CTRL2 = 0;
  TMR3_CNTR2 = 0;
  TMR3_LOAD2 = 0;
  TMR3_COMP12	= 65535;
  TMR3_CMPLD12 = 65535;
  TMR3_SCTRL2 =0;//0
  TMR3_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(2) | TMR_CTRL_LENGTH;


	// configure the pin
  	// Set the pin's alternate function (ALT1 for Timer3)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 1;  // ALT1: QTIMER3_TIMER1
  // Optionally, configure the pad control register to set the electrical characteristics
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x10B0;  
	// configure the input select register
	*Radar_counter.inputselectreg = Radar_counter.inputselectval;

  static IntervalTimer pit_timer_radar;
	pit_timer_radar.begin(gate_timer, GATE_INTERVAL_RADAR); //PIT 0

}

void GPT1_IRQHandler() {

  if (GPT1_SR & GPT_SR_OF1){

    GPT1_SR |= GPT_SR_OF1;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    if (distance_transfer_mode){
        decodeUdpCommands();
        decodeControlCommands();
        dataTransfer();
    }
  }
  
}

void GPT1_Init(){
    // Connect GPS 1PPS signal to pin 25 (EMC_24)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_13 = 1; // GPT1 Capture1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_13 = 0x13000; //Pulldown & Hyst
  CCM_CCGR1 |= CCM_CCGR1_GPT(CCM_CCGR_ON) | CCM_CCGR1_GPT1_SERIAL(CCM_CCGR_ON);
  GPT1_CR = 0;
  GPT1_PR = 0;
  GPT1_SR = 0x3F; // clear all prior status
  GPT1_IR = GPT_IR_OF1IE;
  GPT1_CR = GPT_CR_EN | GPT_CR_CLKSRC(3) | GPT_CR_OM1(0) ;
  GPT1_OCR1 =  data_pulses;
  attachInterruptVector(IRQ_GPT1, GPT1_IRQHandler);
  NVIC_ENABLE_IRQ(IRQ_GPT1);

}

