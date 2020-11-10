#ifndef RADIO_DEFS_H
#define RADIO_DEFS_H

#if __FLOAT_WORD_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error incorrect byte order
#endif

#include <inttypes.h>

#define GETFLAG(i, f) (i & (1 << f))
#define SETFLAG(i, f) (i |= (1 << f))
#define CLRFLAG(i, f) (i &= ~(1 << f))

namespace Radio
{
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

    const uint8_t RESPONSE_PACKET_SIZE = 7;

    const uint8_t RESPONSE_PACKET_FLAGS_OFFSET = 0;
    const uint8_t RESPONSE_PACKET_ANGLE_OFFSET = 2;
    const uint8_t RESPONSE_PACKET_CRC_OFFSET = 6;

    const uint8_t RESPONSE_FLAG_BUSY = 0;

    typedef struct
    {
        uint16_t flags;
        float angle;
        uint8_t crc;
    } ResponsePacket;
} // namespace RADIO

#endif