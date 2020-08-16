#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <PacketSerial.h> //https://github.com/bakercp/PacketSerial.git
#include <FastCRC.h>

namespace Radio
{

#define RADIO_INTERFACE Serial1
    const int RADIO_BAUD_RATE = 38400;
    const uint8_t RADIO_SET_PIN = 2;

#define GETFLAG(i, f) (i & (1 << f))

    typedef union {
        float fp;
        uint8_t bytes[4];
    } FloatUnion;

    const uint8_t PACKET_SIZE = 28;

    const uint8_t PACKET_FLAGS_OFFSET = 0;
    const uint8_t PACKET_A1_OFFSET = 2;
    const uint8_t PACKET_A2_OFFSET = 6;
    const uint8_t PACKET_A3_OFFSET = 10;
    const uint8_t PACKET_S1_OFFSET = 14;
    const uint8_t PACKET_S2_OFFSET = 18;
    const uint8_t PACKET_S3_OFFSET = 22;
    const uint8_t PACKET_CRC_OFFSET = 26;

    typedef struct
    {
        uint16_t flags;
        float a1, a2, a3;
        float s1, s2, s3;
        uint16_t crc;
    } Packet;

    const uint8_t FLAG_ENABLE = 0;
    const uint8_t FLAG_ACK = 1;

    const uint8_t RESPONSE_PACKET_SIZE = 3;

    const uint8_t RESPONSE_PACKET_FLAGS_OFFSET = 0;
    const uint8_t RESPONSE_PACKET_CRC_OFFSET = 2;

    typedef struct
    {
        uint16_t flags;
        uint8_t crc;
    } ResponsePacket;

    void initialize();
    void update();
    void sendStatus();
    boolean packetAvailable();
    Packet getPacket();

} // namespace Radio

#endif