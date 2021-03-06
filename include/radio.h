#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <PacketSerial.h> //https://github.com/bakercp/PacketSerial.git
#include <FastCRC.h>

#include "radio_defs.h"

namespace Radio
{
    #define RADIO_INTERFACE Serial5
    const int RADIO_BAUD_RATE = 115200;
    const uint8_t RADIO_SET_PIN = 35;

    void initialize();
    void update();
    void sendStatus(ResponsePacket);
    boolean packetAvailable();
    Packet getLastPacket();

} // namespace Radio

#endif