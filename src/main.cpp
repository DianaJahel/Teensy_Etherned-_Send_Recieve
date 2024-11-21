#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "unions.h"
#include "dataTransfer.h"
#include "commandsDef.h"
#include "commandRoutines.h"
#include "adcPart.h"
#include "WireSensor.h"


volatile uint32_t ringBufferIsFullCnt = 0;

bool startTCP = false;  // Flag to switch to TCP mode


void PIT_IRQHandler(void)
{
  if (PIT_TFLG0 & 0x1)
  {
    PIT_TFLG0 = 1; 
    
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  }

}
void PIT0_setup(void) {
    CCM_CCGR1 |= (1 << 23);
    PIT_MCR = 0x00;
    PIT_LDVAL0 = 14999999; // 239999;  14999999
    PIT_TCTRL0 |= (1 << 1); //  PIT0 interrupt enable
    PIT_TCTRL0 |= (1 << 0); // PIT0 Timer enable
    attachInterruptVector(IRQ_PIT, PIT_IRQHandler);
    NVIC_SET_PRIORITY(IRQ_PIT, 240);
    NVIC_ENABLE_IRQ(IRQ_PIT);
    
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

  EthernetClient client = TcpServer.accept(); //available();
  //Serial.print(client);
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
  sei();
  initEthernet();
  init_adc1();
  //PIT0_setup();
  interrupts();
  pinMode(LED_BUILTIN, OUTPUT);
  QTIMER3_C1_C0_setup();
}

void loop() {
  
  //renewalConnection();
  decodeUdpCommands();
  decodeControlCommands();
  dataTransfer();

}
