#ifndef DATA_TRANSFER_H
#define DATA_TRANSFER_H

#include "SPI.h"
#include <NativeEthernet.h>
#include <NativeEthernetUDP.h>
#include "dataTransferModeFlagDef.h"
#include "staticIPdef.h"



extern EthernetUDP Udp;                                      // UDP instance
extern EthernetServer TcpServer;                    // TCP server instance
extern EthernetClient dataReceiver;
extern bool networkActive;
extern unsigned long lastRenewalConnectionTime;


const unsigned long RENEWAL_INTERVALL = 800; //ms

#define NUMBER_OF_BUFFERS 8

struct DataPacket {
  unsigned int dataPackNumber;
  float radar;
  unsigned int wireSensorDist;
  float analog1;
  unsigned int analog2;
  unsigned int analog3;
};

extern volatile DataPacket ringBuffer[NUMBER_OF_BUFFERS];

extern volatile int writeIndex;
extern volatile int readIndex;
extern volatile bool bufferFull;

void initEthernet();
void dataTransfer();
void renewalConnection();



#endif //DATA_TRANSFER_H