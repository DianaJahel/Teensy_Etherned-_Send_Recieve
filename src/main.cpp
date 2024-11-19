#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "unions.h"
#include "dataTransfer.h"
#include "commandsDef.h"
#include "commandRoutines.h"

#define NUM_CHANNELS      3
const uint32_t adc_channels[NUM_CHANNELS] = {15, 13, 14}; //Pin 20, 22, 23 on board
volatile uint16_t adc_values[NUM_CHANNELS] = {0};
volatile uint8_t current_channel = 0;


volatile uint32_t ringBufferIsFullCnt = 0;

bool startTCP = false;  // Flag to switch to TCP mode

void PIT_IRQHandler(void)
{
     if (PIT_TFLG0 & 0x1)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      PIT_TFLG0 = 1;              

      
    }

}
void PIT0_setup(void) {
    CCM_CCGR1 |= (1 << 23);
    PIT_MCR = 0x00;
    PIT_LDVAL0 =  1099999; 
    PIT_TCTRL0 |= (1 << 1); //  PIT0 interrupt enable
    PIT_TCTRL0 |= (1 << 0); // PIT0 Timer enable
    attachInterruptVector(IRQ_PIT, PIT_IRQHandler);
    NVIC_ENABLE_IRQ(IRQ_PIT);
}


void createDataPacket() {
  // Generate data and store it in the ring buffer
  DataPacket packet;
  packet.dataPackNumber = 1 + writeIndex;  // Increment for unique value
  packet.radar = 55.2;
  packet.wireSensorDist = 45;
  packet.analog1 = 234.3;
  packet.analog2 = 543;
  packet.analog3 = 234;

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

// Send IP address and TCP port as a response to CMD_DISCOVERY
void sendDiscoveryResponse(IPAddress remoteIP, unsigned int remotePort) {
  uint8_t response[6];
  response[0] = ip[0];
  response[1] = ip[1];
  response[2] = ip[2];
  response[3] = ip[3];
  response[4] = tcpPort & 0xFF;
  response[5] = (tcpPort >> 8) & 0xFF;

  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(response, sizeof(response));
  Udp.endPacket();

  Serial.print("Sent discovery response: IP ");
  Serial.print(ip);
  Serial.print(", Port ");
  Serial.println(tcpPort);
}


unsigned long lastDecodeCmdTime = 0;
unsigned long commandPollingInteval = 100;//nur in dataTransferMode

// Handle incoming TCP connections
void decodeControlCommands() {

  if(dataTransferMode)
  {
    unsigned long now = millis();  
    if(now < lastDecodeCmdTime )  //overflow
      lastDecodeCmdTime = 0;
    if(now - lastDecodeCmdTime < commandPollingInteval)
      return;
    lastDecodeCmdTime = now;
    //_traceNL("decode in datatransfermode");
  }

  EthernetClient client = TcpServer.available();

  if (client) {
    Serial.println("TCP Client connected");

    // Read the CMD_START_DATA_TRANSFER command
    if (client.available()) {
      uint8_t buffer[4];
      int len = client.read(buffer, 4);

      if (len == 4) {
        // Decode cmdID and cmdSz
        uint16_t cmdID = buffer[0] | (buffer[1] << 8);
        uint16_t cmdSz = buffer[2] | (buffer[3] << 8);

        Serial.print("TCP Received cmdID: ");
        Serial.println(cmdID);
        Serial.print("TCP Received cmdSz: ");
        Serial.println(cmdSz);

        switch (cmdID)
        {
          case CMD_ECHO:
          Serial.println("CMD_ECHO");
          break;
        case CMD_RESET_RADAR_COUNTER:
          Serial.println("CMD_RESET_RADAR_COUNTER");
          
          break;
        case CMD_START_DATA_TRANSFER:
          dataReceiver = client;
          startDataTransferCmd();
          break;
        case CMD_STOP_DATA_TRANSFER:
          Serial.println("CMD_STOP_DATA_TRANSFER");
          break;
        case CMD_DATA_PACKET_FORMAT_DESCRIPTION:
          Serial.println("CMD_DATA_PACKET_FORMAT_DESCRIPTION");
          break;
        default:
          char strBuff[24];
          sprintf(strBuff, "unbekannte cmdID: %hu\n", cmdID );
          
          break;

        }

      }
    }

    //client.stop();
    //Serial.println("TCP Client disconnected");
  }
}




void decodeUdpCommands() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = '\0';
    }

    uint16_t cmdID = (packetBuffer[1] << 8) | packetBuffer[0];  // Command ID

    Serial.print("UDP Received Command ID: ");
    Serial.println(cmdID);

    if (cmdID == CMD_DISCOVERY) {
      sendDiscoveryResponse(Udp.remoteIP(), Udp.remotePort());
    }
  }
}



void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  initEthernet();
  init_adc1();
  //PIT0_setup();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  
  renewalConnection();
  decodeUdpCommands();
  decodeControlCommands();
  dataTransfer();

}
