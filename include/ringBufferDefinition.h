#ifndef RING_BUFF_DEFINITION_H
#define RING_BUFF_DEFINITION_H


#include "dtkCommunicationBuffer.h"

//der Puffer ist für Datenerfassung und Senden an PC gedacht

const int NUMBER_OF_DATA_BUFFERS = 8; 
extern dtkCommunicationBuffer communicationBuffer[NUMBER_OF_DATA_BUFFERS];

//welcher von den Puffern aktuell fürs Schreiben ist
extern volatile byte currentWriteDataBuffer;
//welcher von den Puffern aktuell fürs Senden übers Netzwerk ist
extern volatile byte currentReadDataBuffer;

#endif //RING_BUFF_DEFINITION_H
