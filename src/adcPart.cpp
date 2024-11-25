#include <Arduino.h>
#include "adcPart.h"
#include "dataTransfer.h"
#include "WireSensor.h"
#include "radarSensor.h"

#define NUM_CHANNELS      3
const uint32_t adc_channels[NUM_CHANNELS] = {15, 13, 14}; //Pin 20, 22, 23 on board
volatile uint16_t adc_values[NUM_CHANNELS] = {0};
volatile uint8_t current_channel = 0;

volatile int global_packet_nmbr = 0;
void createDataPacket() {
  // Generate data and store it in the ring buffer
  DataPacket packet;
  packet.dataPackNumber = global_packet_nmbr++;  // Increment for unique value

  if (count_update) {
    //Serial.printf("%8u\n", count_output * MULT_FACTOR);
    Freq= count_output * MULT_FACTOR;
    //Serial.print(Freq);
    //Serial.println(" Hz ");
    count_update = false;
  } 
  packet.radar = Freq;
  packet.wireSensorDist = read_wire_sensor();
  packet.analog1 = adc_values[0];
  packet.analog2 = adc_values[1];
  packet.analog3 = adc_values[2];
  //packet.readwriteDiff = readIndex- writeIndex;

  // Write data to the ring buffer (cast volatile away for assignment)
  ((DataPacket&)ringBuffer[writeIndex]) = packet;
  writeIndex = (writeIndex + 1) % NUMBER_OF_BUFFERS;

  if (writeIndex == readIndex) {
    bufferFull = true;  // Mark the buffer as full
  }
}

void start_adc_conversion(void) {
      // Set the next ADC channel and start conversion
    ADC1_HC0 = (adc_channels[current_channel] & 0xF) | ADC_HC_AIEN;

}

void ADC1_IRQHandler(void) {
 

  if (ADC1_HS & ADC_HS_COCO0) {
      // Store the conversion result
      adc_values[current_channel] = ADC1_R0;

      // Move to the next channel
      current_channel++;
      if (current_channel > NUM_CHANNELS) {
          current_channel = 0;
          createDataPacket();
      }

      // Start the next conversion
      start_adc_conversion();
  }

}

void init_adc1(void) {
// Enable clock gating for ADC1
    CCM_CCGR1 |= CCM_CCGR1_ADC1(CCM_CCGR_ON);
    ADC1_GC = ADC_CFG_AVGS(1); //8 samples avg
    ADC1_HC0 = adc_channels[0] & 0x3f; //start first conversion

     // Enable ADC interrupts
    ADC1_HC0 |= ADC_HC_AIEN;
    attachInterruptVector(IRQ_ADC1, ADC1_IRQHandler);
    // Enable ADC interrupt in NVIC (Interrupt #57 for ADC1)
    NVIC_ISER2 |= (1 << (67 - 64)); // 67
    start_adc_conversion();
}