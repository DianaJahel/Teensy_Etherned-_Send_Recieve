//Since the Watchdog functionality of the radar sensor is made dy the PIT1, part of the speed calcuation is done on the void PIT_IRQHandler(void) 
//can be found on periodicTimer.cpp

#include <Arduino.h>
#include "radarSensor.h"

volatile bool count_update = false;
volatile uint32_t count_output=0;

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
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x10B0;  // Example value for pad control (e.g., pull resistors, drive strength, etc.)
	// configure the input select register
	*Radar_counter.inputselectreg = Radar_counter.inputselectval;

  static IntervalTimer pit_timer_radar;
	pit_timer_radar.begin(gate_timer, GATE_INTERVAL_RADAR); //PIT 0

}



void QT1_IRQHandler(void){


  // Check and clear the interrupt flag
  if (TMR1_SCTRL0 & (1 << 15)) // Check TCF (Timer Compare Flag)
  {

      mycounter++;
      if (mycounter>4000000){
        //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        //Serial.println( mycounter);
          mycounter = 0;
      }
      //Serial.println( mycounter);
      TMR1_CSCTRL0 &= ~(TMR_CSCTRL_TCF1);  // clear

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
 

}

/*void TimerInit_Init(void){
  // turn on clock to the specific quad timer (QuadTimer3)
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);
  TMR1_CTRL0 |= 0x20;
  TMR1_SCTRL0 |= 0x00;
  TMR1_LOAD0 = 0x00;
  TMR1_COMP10 = 938;
  TMR1_COMP20 = 0x00;
  TMR1_CMPLD10 = 938;
  TMR1_CTRL0 |= TMR_CTRL_PCS(0xF) | TMR_CTRL_LENGTH  | TMR_CTRL_CM(0x01);
  //TMR1_CTRL0 |=TMR_CTRL_DIR;
  TMR1_CNTR0 = 0x00;
  TMR1_SCTRL0 |= TMR_SCTRL_TCFIE;
  TMR1_CSCTRL0 |= 0x41;
  attachInterruptVector(IRQ_QTIMER1, QT1_IRQHandler);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

}*/