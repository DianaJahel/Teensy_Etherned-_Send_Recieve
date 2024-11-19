#ifndef COMMANDSDEF_H
#define COMMANDSDEF_H

//Format der Befehle int16/*Befehlscode*/|int16/*Größe der Daten in Puffer in Bytes*/|byte[] Puffer

  //Befehle
const int CMD_ECHO = 0x0001;
const int CMD_START_DATA_TRANSFER = 0x0002;
const int CMD_STOP_DATA_TRANSFER = 0x0003;
const int CMD_RESET_RADAR_COUNTER = 0x0004;
const int CMD_DISCOVERY = 0x0005;  //Info über den Controller ausgeben
const int CMD_DATA_PACKET_FORMAT_DESCRIPTION = 0x0006;  //Wie sieht ein Datenpaket aus? (übertragene Informationen und deren Reihenfolge, Details an der Stelle, wo der Befehl dekodiert wird)
  //Antworten
const int  CMD_ANSWER_OK = 0x8000;
const int  CMD_ANSWER_ERROR = 0x8001;
const int  CMD_START_TRANSFER_ACCEPTED = 0x8002;
const int  CMD_START_TRANSFER_REFUSED = 0x8003;
const int  CMD_DISCOVERY_ANSWER = 0x8004;
const int  CMD_DATA_TRANSFER_STOPPED = 0x8005;
const int CMD_DATA_PACKET_FORMAT_DESCRIPTION_ANSWER = 0x8006;
//undefinierter Befehl, wird an PC-Seite verwendet, wenn irgendwas nicht stimmt
const int  CMD_UNDEFINED = 0xffff;

#endif  //#ifndef COMMANDSDEF_H
