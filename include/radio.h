#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <PacketSerial.h> //https://github.com/bakercp/PacketSerial.git
#include <FastCRC.h>

#include "radio_defs.h"

namespace Radio
{
    #define RADIO_INTERFACE Serial1
    const int RADIO_BAUD_RATE = 115200;
    const uint8_t RADIO_SET_PIN = 2;

    void initialize();
    void update();
    void sendStatus(uint16_t);
    boolean packetAvailable();
    Packet getLastPacket();

} // namespace Radio

#endif