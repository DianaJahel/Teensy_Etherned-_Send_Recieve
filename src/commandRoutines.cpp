#include "commandRoutines.h"
#include "dataTransfer.h"
#include "ringBufferDefinition.h"
#include "unions.h"
#include "commandsDef.h"

void startDataTransferCmd()
{
  Serial.println("startDataTransferCmd");

  if(dataTransferMode)
  {
    Serial.println("already in transfer mode");
    return;
  }
  if(dataReceiver.connected())
  {
    Serial.print("client: ");
    Serial.print(dataReceiver.remoteIP());
    Serial.print(":");
    Serial.println(dataReceiver.remotePort());
    for(byte i = 0; i < NUMBER_OF_DATA_BUFFERS; i++ )
      communicationBuffer[i].reset();    

    currentWriteDataBuffer = 0;
    currentReadDataBuffer = 0;

    uint16Data dt0;
    dt0.value = CMD_START_TRANSFER_ACCEPTED;
    uint16Data dt1;
    dt1.value = 0;
    char answBuff[4];
    answBuff[0] = dt0.array[0];
    answBuff[1] = dt0.array[1];
    answBuff[2] = dt1.array[0];
    answBuff[3] = dt1.array[1];
    dataReceiver.write(answBuff, sizeof(answBuff));
    Serial.println("Sent CMD_START_TRANSFER_ACCEPTED");

    delay(1);
    dataTransferMode = true;
  }
  else
  {
      Serial.print("No client connected");
  }
}