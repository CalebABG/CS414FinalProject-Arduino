#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "arduino_packet.h"

typedef struct ArduinoStateMachine_t
{
    uint32_t state = 0;
    uint32_t dataIndex = 0;

    void loop(int16_t dataRead, ArduinoPacket* packet, void (*processPacketFunc)())
    {
        /*
         State values correspond to the byte indexes for the
         packet format. Look to the packet structure as a guide for
         how these values were obtained.
        */
        switch (state)
        {
        case 0:
            if (dataRead == START_OF_PACKET)
                state++;
            else
                state = 0;
            break;

        case 1:
            packet->id = (uint16_t)dataRead;
            state++;
            break;

        case 2:
            packet->id += ((uint16_t)dataRead) << 8;
            state++;
            break;

        case 3:
            packet->crc = ((uint32_t)dataRead);
            state++;
            break;

        case 4:
            packet->crc += ((uint32_t)dataRead) << 8;
            state++;
            break;

        case 5:
            packet->crc += ((uint32_t)dataRead) << 16;
            state++;
            break;

        case 6:
            packet->crc += ((uint32_t)dataRead) << 24;
            state++;
            break;

        case 7:
            if (dataRead == 0x1 || dataRead == 0x0)
            {
                packet->ack = dataRead;
                state++;
            }
            else
            {
                state = 0;
            }
            break;

        case 8:
            packet->dlc = dataRead;
            state++;
            break;

        case 9:
            packet->dlc += ((uint16_t)dataRead) << 8;
            state++;
            break;

        case DATA_START_INDEX ...(DATA_START_INDEX + (DATA_LENGTH - 1)):
            packet->data[dataIndex++] = dataRead;
            state++;
            break;

        case (DATA_START_INDEX + DATA_LENGTH):
            if (dataRead == END_OF_PACKET)
                processPacketFunc();

            state = 0;
            dataIndex = 0;
            packet->clearData();
            break;
        }
    }
} ArduinoStateMachine;
