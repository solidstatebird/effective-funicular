#include <Arduino.h>
#include <PacketSerial.h>
#include <FastCRC.h>

#include "radio.h"

using namespace Radio;

void processPacket(const void *, const uint8_t *, size_t);
void writeFloat(float, uint8_t *, const size_t);
float readFloat(const uint8_t *, const size_t);

PacketSerial packetInterface;
Packet packet;
FastCRC16 crc16;
FastCRC8 crc8;
boolean newData = false;

void Radio::initialize()
{
    RADIO_INTERFACE.begin(RADIO_BAUD_RATE);
    packetInterface.setStream(&RADIO_INTERFACE);
    packetInterface.setPacketHandler(&processPacket);
    pinMode(RADIO_SET_PIN, OUTPUT);
    digitalWrite(RADIO_SET_PIN, HIGH);
    delay(250);
}

void Radio::sendStatus()
{
    uint8_t data[RESPONSE_PACKET_SIZE];
    uint16_t flags = 0;
    data[RESPONSE_PACKET_FLAGS_OFFSET] = flags >> 8;;
    data[RESPONSE_PACKET_FLAGS_OFFSET+1] = flags;
    uint8_t packetcrc = crc8.smbus(data, RESPONSE_PACKET_SIZE - 1);
    data[RESPONSE_PACKET_CRC_OFFSET] = packetcrc;

    packetInterface.send(data, RESPONSE_PACKET_SIZE);
}

void Radio::update()
{
    packetInterface.update();
}

boolean Radio::packetAvailable()
{
    return newData;
}

Packet Radio::getPacket()
{
    newData = false;
    return packet;
}

//hidden functions

void processPacket(const void *sender, const uint8_t *data, size_t size)
{
    if (sender != &packetInterface)
        return;
    if (size != PACKET_SIZE)
        return;

    uint16_t calc_crc = crc16.ccitt(data, size - 2);
    uint16_t rx_crc = (data[PACKET_CRC_OFFSET] << 8) + data[PACKET_CRC_OFFSET+1];
    if (calc_crc != rx_crc)
        return;

    packet.flags = (data[PACKET_FLAGS_OFFSET] << 8) + data[PACKET_FLAGS_OFFSET+1];
    packet.a1 = readFloat(data, PACKET_A1_OFFSET);
    packet.a2 = readFloat(data, PACKET_A2_OFFSET);
    packet.a3 = readFloat(data, PACKET_A3_OFFSET);
    packet.s1 = readFloat(data, PACKET_S1_OFFSET);
    packet.s2 = readFloat(data, PACKET_S2_OFFSET);
    packet.s3 = readFloat(data, PACKET_S3_OFFSET);
    packet.crc = calc_crc;

    newData = true;
}

#if __FLOAT_WORD_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error incorrect byte order
#endif

//converts little-endian to big-endian
void writeFloat(float value, uint8_t *data, size_t index)
{
    FloatUnion input;
    input.fp = value;

    data[index + 0] = input.bytes[3];
    data[index + 1] = input.bytes[2];
    data[index + 2] = input.bytes[1];
    data[index + 3] = input.bytes[0];
}

//converts big-endian to little-endian
float readFloat(const uint8_t *data, const size_t index)
{
    FloatUnion output;

    output.bytes[0] = data[index + 3];
    output.bytes[1] = data[index + 2];
    output.bytes[2] = data[index + 1];
    output.bytes[3] = data[index + 0];

    return output.fp;
}