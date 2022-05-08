#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <CRC32.h>

/* Needed Packet section and data length defines */
#define START_OF_PACKET 0x1
#define END_OF_PACKET 0x4
#define DATA_LENGTH 0x10

/*
NOTE: 
Update this start index if packet format is changed 
This index is based off the byte for the Start of Packet +
the number of bytes of the Header of the packet
*/
#define DATA_START_INDEX 0xA

/* Packet IDs - add more as needed */
#define SENSOR_DATA_ID 0xD7
#define DRIVE_PARAMETERS_ID 0xD8
#define STOP_MOTORS_ID 0xE0
#define PARENTAL_OVERRIDE_ID 0xDF

typedef struct ArduinoPacket_t
{
    /* Start of Packet */
    uint8_t sop = START_OF_PACKET;

    /* Header */
    uint16_t id;
    uint32_t crc;
    bool ack;

    /* Data */
    uint16_t dlc;
    uint8_t data[DATA_LENGTH] = {0};

    /* End of Packet */
    uint8_t eop = END_OF_PACKET;

    void clearData()
    {
        for (uint16_t i = 0; i < DATA_LENGTH; ++i)
            data[i] = 0;
    }

    /* CRC consists of: id, ack, dlc, data */
    uint32_t calculateCRC()
    {
        CRC32 crc;

        crc.update((uint8_t)id);
        crc.update((uint8_t)(id >> 8));
        crc.update((uint8_t)ack);
        crc.update((uint8_t)dlc);
        crc.update((uint8_t)(dlc >> 8));
        crc.update(data, DATA_LENGTH);

        return crc.finalize();
    }

    bool crcOk()
    {
        return crc == calculateCRC();
    }

    /* Reference: https://stackoverflow.com/questions/3991478/building-a-32-bit-float-out-of-its-4-composite-bytes */
    float getFloat(uint16_t startIndex)
    {
        float f;
        uint8_t *fPtr = (uint8_t *)&f;

        // Little Endian
        fPtr[0] = data[startIndex];
        fPtr[1] = data[startIndex + 1];
        fPtr[2] = data[startIndex + 2];
        fPtr[3] = data[startIndex + 3];

        return f;
    }

    int16_t getInt16(uint32_t startIndex)
    {
        // Little Endian
        return data[startIndex] +
               (((uint16_t)data[startIndex + 1]) << 8);
    }

    int32_t getInt32(uint32_t startIndex)
    {
        // Little Endian
        return data[startIndex] +
               (((uint32_t)data[startIndex + 1]) << 8) +
               (((uint32_t)data[startIndex + 2]) << 16) +
               (((uint32_t)data[startIndex + 3]) << 24);
    }

} ArduinoPacket;
