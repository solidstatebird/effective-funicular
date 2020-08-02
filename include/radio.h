#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <PacketSerial.h> //https://github.com/bakercp/PacketSerial.git
#include <FastCRC.h>

#define RADIO_INTERFACE Serial1
const int RADIO_BAUD_RATE = 38400;
const uint8_t RADIO_SET_PIN = 2;

#define GETFLAG(i, f) (i & (1 << f))

typedef union 
{
    float fp;
    uint8_t bytes[4];
} FloatUnion;

#define PACKET_SIZE 28

#define PACKET_FLAGS_OFFSET 0
#define PACKET_A1_OFFSET 2
#define PACKET_A2_OFFSET 6
#define PACKET_A3_OFFSET 10
#define PACKET_S1_OFFSET 14
#define PACKET_S2_OFFSET 18
#define PACKET_S3_OFFSET 22
#define PACKET_CRC_OFFSET 26

typedef struct
{
    uint16_t flags;
    float a1, a2, a3;
    float s1, s2, s3;
    uint16_t crc;
} Packet;

#define FLAG_ENABLE 0
#define FLAG_ACK 1

#define RESPONSE_PACKET_SIZE 3

#define RESPONSE_PACKET_CRC_OFFSET 2

typedef struct
{
    uint16_t flags;
    uint8_t crc;
} ResponsePacket;

void radioInitialize();
void radioUpdate();
void radioSendStatus();
boolean radioPacketAvailable();
Packet radioGetPacket();

#endif