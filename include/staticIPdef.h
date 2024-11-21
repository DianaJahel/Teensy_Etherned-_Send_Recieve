#ifndef STATIC_IP_DEF_H
#define STATIC_IP_DEF_H

#include "NativeEthernet.h"

//const IPAddress ip(192, 168, 2, 23);                        // Static IP address
const IPAddress ip(192, 168, 1, 23);                        // Static IP address
const int udpPort = 12345;                         // UDP port
const int tcpPort = 54321;                         // TCP port

#endif  //STATIC_IP_DEF_H