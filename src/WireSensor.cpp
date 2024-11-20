#include <Arduino.h>
#include "WireSensor.h"


float calculo =0.0;
float wire_dist =0.0;
float edges_per_meter= 40.0332;

typedef struct {
	IMXRT_TMR_t *timer;
	int timerchannel;
	int pin;
	int pinconfig;
	volatile uint32_t *inputselectreg;
	int inputselectval;
} timerinfo_t;

const timerinfo_t PHASE_A = { &IMXRT_TMR3, 1, 18, 1, &IOMUXC_QTIMER3_TIMER1_SELECT_INPUT, 0 };//usually in other pins is 1 instead of 0
const timerinfo_t PHASE_B = {&IMXRT_TMR3, 0,  19,  1, &IOMUXC_QTIMER3_TIMER0_SELECT_INPUT, 1};

float read_wire_sensor() {
    uint16_t count = PHASE_A.timer->CH[PHASE_A.timerchannel].CNTR;
    calculo= count/edges_per_meter;
	return calculo;
}

void QTIMER3_C1_C0_setup(void){  //Quad encoder  Wire sensor
  // turn on clock to the specific quad timer (QuadTimer3)
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
  CCM_CCGR4 |= CCM_CCGR4_IOMUXC(CCM_CCGR_ON);

  TMR3_CTRL1 = 0;
  TMR3_CNTR1 = 0;
  TMR3_LOAD1 = 0;
  TMR3_COMP11	= 65535;
  TMR3_CMPLD11 = 65535;
  TMR3_SCTRL1 = TMR_SCTRL_TOF | TMR_SCTRL_TOFIE |TMR_SCTRL_IEF |TMR_SCTRL_IEFIE;//0
  TMR3_CTRL1 = TMR_CTRL_CM(0b100) | TMR_CTRL_PCS(1)  | TMR_CTRL_LENGTH;

    
  TMR3_CTRL0 = 0;
  TMR3_CNTR0 = 0;
  TMR3_LOAD0 = 0;
  TMR3_COMP10	= 65535;
  TMR3_CMPLD10 = 65535;
  TMR3_SCTRL0= TMR_SCTRL_TOF | TMR_SCTRL_TOFIE |TMR_SCTRL_IEF |TMR_SCTRL_IEFIE;//0
  TMR3_CTRL0 = TMR_CTRL_CM(0b100) | TMR_CTRL_SCS(1) | TMR_CTRL_LENGTH;


	// configure the pin
  // Set the pin's alternate function (ALT1 for Timer3)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01 = 1;  // ALT1: QTIMER3_TIMER1
  // Optionally, configure the pad control register to set the electrical characteristics
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_01 = 0x10B0;  // Example value for pad control (e.g., pull resistors, drive strength, etc.)

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_00 = 0x10B0;

	// configure the input select register
  *PHASE_A.inputselectreg = PHASE_A.inputselectval;
  *PHASE_B.inputselectreg = PHASE_B.inputselectval;

}
