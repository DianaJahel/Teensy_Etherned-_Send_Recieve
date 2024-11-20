#include "dataTransfer.h"
#include "SPI.h"
#include <NativeEthernet.h>
#include <NativeEthernetUDP.h>
#include "dataTransferModeFlagDef.h"
#include "staticIPdef.h"
#include "ringBufferDefinition.h"


EthernetUDP Udp;                                      // UDP instance
EthernetServer TcpServer(tcpPort);                    // TCP server instance
EthernetClient dataReceiver;
bool networkActive = false;


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

unsigned long lastRenewalConnectionTime = 0;

void renewalConnection()
{
  unsigned long now = millis();
  if(now < lastRenewalConnectionTime )
  {
    //overflow
    lastRenewalConnectionTime  = 0;
  }
  if( (now-lastRenewalConnectionTime) > RENEWAL_INTERVALL )
  {
    lastRenewalConnectionTime = now;
      
    byte res = Ethernet.maintain();
    switch(res)
    {
      case 0:
      //Serial.print("renewal connection, nothing happened");
      //_traceNL("renewal connection, nothing happened");
        break;
      case 1:
        //_traceNL("renewal failed");
        break;
      case 2:
        //_traceNL("renewal success");
        break;
      case 3:
        //_traceNL("rebind fail");
        break;
      case 4:
        //_traceNL("rebind success");
        break;
    }
  }
}

void dataTransfer() {

  if (!dataTransferMode|| !dataReceiver.connected()) {
    dataTransferMode = false;
    //Serial.println("TCP client disconnected");
    return;
  }

  while (readIndex != writeIndex || bufferFull) {
    // Send data from the ring buffer
    DataPacket packet = ((DataPacket&)ringBuffer[readIndex]);
    dataReceiver.write((uint8_t*)&packet, sizeof(packet));
    Serial.println("Data packet sent");

    readIndex = (readIndex + 1) % NUMBER_OF_BUFFERS;
    bufferFull = false;  // Reset buffer full flag after reading
  }
}