#ifndef DTK_COMMUNICATION_BUFFER_H
#define DTK_COMMUNICATION_BUFFER_H

#include <Arduino.h>

struct dtkCommunicationBuffer
{
  static const int BUFF_SZ = 600;
  //optimiert für Leistung, daher nicht besonders elegant ;)
  byte buffer[BUFF_SZ];
  //lock == true -> kein Schreiben mehr in den Puffer, kann versendet werden, Senderoutine setzt das Flag wieder auf false, wenn alle Daten aus dem Puffer versendet wurden.
  volatile bool lock = false;
  //aktuelle Byteposition, an der geschrieben wird
  volatile int writePtr = 0;

  void reset()
  {
    lock = false;
    writePtr = 0;  
  }
};

#endif // #ifndef DTK_COMMUNICATION_BUFFER_H
