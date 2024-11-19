#include "ringBufferDefinition.h"

 dtkCommunicationBuffer communicationBuffer[NUMBER_OF_DATA_BUFFERS];

//welcher von den Puffern aktuell fürs Schreiben ist
volatile byte currentWriteDataBuffer = 0;
//welcher von den Puffern aktuell fürs Senden übers Netzwerk ist
volatile byte currentReadDataBuffer = 0;
