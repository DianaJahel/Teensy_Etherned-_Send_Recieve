#include <Arduino.h>
#include "watchDog1.h"

void WDOG1_IRQHandler(void) {
  if(WDOG1_WICR &((1 << 14))){
    //digitalWrite(LED_BUILTIN,HIGH);
    WDOG1_WICR  |= (1 << 14); //Clear interrupt

  }

}

void WDOG_init(){

  CCM_CCGR3 |= CCM_CCGR3_WDOG1(CCM_CCGR_ON);
  //PDE bit of Watchdog Miscellaneous Control (WMCR) should be cleared to disable the power down counter.
  WDOG1_WMCR &= ~ (0<<0);
  //WT field of Watchdog Control (WCR) should be programmed for sufficient timeout value.
  WDOG1_WCR |=  WDOG_WCR_WDE;
  //WDOG should be enabled by setting WDE bit of Watchdog Control (WCR) so that the timeout counter
   WDOG1_WCR |= WDOG_WCR_WT(10);
  WDOG1_WICR |= (1<<15);
  attachInterruptVector(IRQ_WDOG1, WDOG1_IRQHandler);  
  NVIC_ISER2 |= (1 << (92 - 64)); // 67

}