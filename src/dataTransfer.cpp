#include "dataTransfer.h"
#include "SPI.h"
#include <NativeEthernet.h>
#include <NativeEthernetUDP.h>
#include "dataTransferModeFlagDef.h"
#include "staticIPdef.h"
#include "ringBufferDefinition.h"
#include "commandsDef.h"
#include "commandRoutines.h"
#include "unions.h"
#include "radarSensor.h"


EthernetUDP Udp;                                      // UDP instance
EthernetServer TcpServer(tcpPort);                    // TCP server instance
EthernetClient dataReceiver;
bool networkActive = false;

volatile uint32_t ringBufferIsFullCnt = 0;

bool startTCP = false;  // Flag to switch to TCP mode

volatile DataPacket ringBuffer[NUMBER_OF_BUFFERS];
volatile int writeIndex = 0;
volatile int readIndex = 0;
volatile bool bufferFull = false;

void initEthernet(){
    
    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // MAC address

  // Initialize Ethernet and servers
  Ethernet.begin(mac, ip);
  Udp.begin(udpPort);
  TcpServer.begin();

  Serial.print("Server IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("UDP listening on port: ");
  Serial.println(udpPort);
  Serial.print("TCP listening on port: ");
  Serial.println(tcpPort);



}


void modifyFreqCmnd(uint8_t Freq_A, uint8_t Freq_B){

  unsigned int result = pow(10, Freq_B);
  unsigned int New_Freq= Freq_A*result;
  Serial.println(New_Freq);

  unsigned int conv_fac = pow(10, 6);
  unsigned long update_time= (conv_fac/New_Freq);
  Serial.println(update_time);
  distance_transfer_mode =false;
  pit_timer_sendData.update(update_time); 

  uint16Data dt0;
  dt0.value = CMD_FREQUENCY_ACCEPTED;
  uint16Data dt1;
  dt1.value = 0;
  char answBuff[4];
  answBuff[0] = dt0.array[0];
  answBuff[1] = dt0.array[1];
  answBuff[2] = dt1.array[0];
  answBuff[3] = dt1.array[1];
  dataReceiver.write(answBuff, sizeof(answBuff));
  Serial.println("Sent CMD_FREQUENCY_ACCEPTED");

    

}


void modifyDataTransferMode(uint8_t Num_A, uint8_t Num_B){

  unsigned int result = pow(10, Num_B);
  unsigned int update_pulses= Num_A*result;
  Serial.println(update_pulses);


  distance_transfer_mode = true;
  GPT1_OCR1 = update_pulses;
  

  uint16Data dt0;
  dt0.value = CMD_DISTANCE_DATA_TRANSFER_ACCEPTED;
  uint16Data dt1;
  dt1.value = 0;
  char answBuff[4];
  answBuff[0] = dt0.array[0];
  answBuff[1] = dt0.array[1];
  answBuff[2] = dt1.array[0];
  answBuff[3] = dt1.array[1];
  dataReceiver.write(answBuff, sizeof(answBuff));
  Serial.println("Sent CMD_DISTANCE_DATA_TRANSFER_ACCEPTED");

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
        Serial.println(buffer[2]);
        Serial.println(buffer[3]);

        switch (cmdID)
        {
        case CMD_ECHO:
          Serial.println("CMD_ECHO");
          break;
        case CMD_FREQUENCY:
          dataReceiver = client;
          Serial.println("CMD_FREQUENCY");
          modifyFreqCmnd(buffer[2],buffer[3]);

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
        case CMD_DISTANCE_DATA_TRANSFER:
          dataReceiver = client;
          Serial.println("CMD_DISTANCE_DATA_TRANSFER");
          modifyDataTransferMode(buffer[2],buffer[3]);
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

void dataTransfer() {
  // if(!dataReceiver.connected())
  //   Serial.println("TCP client disconnected (Diana)");

  if (!dataTransferMode|| !dataReceiver.connected()) {
    dataTransferMode = false;
    //Serial.println("TCP client disconnected");
    return;
  }

  while (readIndex != writeIndex || bufferFull) {
    //if(bufferFull)
      //Serial.println("Data packet sent: buffer full");

    // Send data from the ring buffer
    DataPacket packet = ((DataPacket&)ringBuffer[readIndex]);
    dataReceiver.write((uint8_t*)&packet, sizeof(packet));
    //Serial.println("Data packet sent");

    readIndex = (readIndex + 1) % NUMBER_OF_BUFFERS;
    bufferFull = false;  // Reset buffer full flag after reading
  }
}