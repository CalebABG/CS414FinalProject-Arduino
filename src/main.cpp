#include <Arduino.h>
#include <AltSoftSerial.h>
#include <CRC32.h>

// Define functions
#define sprint(x) Serial.print(x)
#define sprintln(x) Serial.println(x)

#define START_BYTE 0x1
#define END_BYTE 0x4
#define DATA_LENGTH 0x40

#define SENSOR_DATA_ID 0xD7
#define STOP_MOTORS_ID 0xE0
#define PARENTAL_OVERRIDE_ID 0xDF

#define MIN_MOTOR_SPEED 0
#define MAX_MOTOR_SPEED 255

// H-Bridge Pins
#define EnA 3
#define EnB 11

// Motor A
#define In1 4
#define In2 7

// Motor B
#define In3 2
#define In4 12

enum Motor
{
    LEFT,
    RIGHT
};

/*
NOTES:

--------------

Number Data Types - https://www.arduino.cc/en/reference/int

On the Arduino Uno (and other ATMega based boards) 
    int stores a 16-bit (2-byte) value.

On the Arduino Due and SAMD based boards (like MKR1000 and Zero), 
    int stores a 32-bit (4-byte) value.

--------------

Arduino is Little Endian - https://forum.arduino.cc/t/little-endian-or-big-endian/41382

--------------

57600 baud = 7200 bytes per second

*/

typedef struct GoPacket_t
{
    byte startByte = START_BYTE; // 0
    uint16_t id; // 1 - 2
    uint32_t crc32; // 3 - 6
    bool ack; // 7
    uint16_t dataLength; // 8 - 9
    byte data[DATA_LENGTH] = {0}; // 10 - 73
    byte endByte = END_BYTE; // 74

} GoPacket;

CRC32 crc;
AltSoftSerial bluetooth;

uint64_t timeNow = 0;
uint32_t safetyTimer = 200;
bool stoppedMotors = false;

uint32_t state = 0;

GoPacket receivedPacket;
byte packetDataIndex = 0;

// Functions

// CRC32 consists of: id, ack, dataLength, data
uint32_t calculatePacketCRC32(GoPacket* packet)
{
    crc.update(packet->id & 0xFF); // Low byte
    crc.update((packet->id >> 8) & 0xFF); // High byte

    crc.update(packet->ack);

    crc.update(packet->dataLength & 0xFF);
    crc.update((packet->dataLength >> 8) & 0xFF);

    crc.update(packet->data, DATA_LENGTH);

    uint32_t calcCRC32 = crc.finalize();
    crc.reset();

    return calcCRC32;
}

bool packetCRC32Match(uint32_t receivedPacketCRC32, uint32_t calculatedPacketCRC32)
{
    return receivedPacketCRC32 == calculatedPacketCRC32;
}

int16_t getInt16FromPacketData(byte* packetData, uint32_t startIndex)
{
    // Little Endian
    return packetData[startIndex] + 
           (packetData[startIndex + 1] << 8);
}

int32_t getInt32FromPacketData(byte* packetData, uint32_t startIndex)
{
    // Little Endian
    return packetData[startIndex] + 
           (packetData[startIndex + 1] << 8) + 
           (packetData[startIndex + 2] << 16) + 
           (packetData[startIndex + 3] << 24);
}

void zeroOutPacketData()
{
    for (byte i = 0; i < DATA_LENGTH; ++i)
        receivedPacket.data[i] = 0;
}

void setupMotors()
{
    pinMode(EnA, OUTPUT);
    pinMode(EnB, OUTPUT);

    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);
    pinMode(In3, OUTPUT);
    pinMode(In4, OUTPUT);
}

void setMotorSpeed(Motor motor, int16_t motorSpeed)
{
    uint16_t motorEnaPin,
        motorAPin,
        motorBPin;

    switch (motor)
    {
    case LEFT:
        motorEnaPin = EnA;
        motorAPin = In1;
        motorBPin = In2;
        break;

    case RIGHT:
        motorEnaPin = EnB;
        motorAPin = In3;
        motorBPin = In4;
        break;
    }

    // Reverse
    if (motorSpeed < 0)
    {
        digitalWrite(motorAPin, HIGH);
        digitalWrite(motorBPin, LOW);
    }
    // Forward
    else if (motorSpeed > 0)
    {
        digitalWrite(motorAPin, LOW);
        digitalWrite(motorBPin, HIGH);
    }
    // Stop
    else
    {
        digitalWrite(motorAPin, LOW);
        digitalWrite(motorBPin, LOW);
    }

    analogWrite(motorEnaPin, (uint16_t)constrain(abs(motorSpeed), MIN_MOTOR_SPEED, MAX_MOTOR_SPEED));
}

void setMotorSpeeds(int16_t leftMotorSpeed, int16_t rightMotorSpeed)
{
    setMotorSpeed(LEFT, leftMotorSpeed);
    setMotorSpeed(RIGHT, rightMotorSpeed);
}

float scaleSensorX(int16_t v)
{
    return .40 * v;
}

float scaleSensorY(int16_t v)
{
    return v * .115;
}

void safetyIsrMotors()
{
    if (millis() > timeNow + safetyTimer)
    {
        timeNow = millis();

        if (receivedPacket.ack)
        {
            stoppedMotors = false;
        }
        else if (!stoppedMotors)
        {
            // sprintln("Safety: Stopping Motors");
            setMotorSpeeds(0x0, 0x0);
            stoppedMotors = true;
        }

        receivedPacket.ack = false;
    }
}

void processStopMotors()
{
    sprintln("Stopping Motors");
    setMotorSpeeds(0x0, 0x0);
}

void processParentalOverride()
{
    sprintln("Parental Override");
}

void processSensorData(int16_t sensorX, int16_t sensorY)
{
    int16_t scaledSensorX = (int16_t)floor(scaleSensorX(sensorX));
    int16_t scaledSensorY = (int16_t)floor(scaleSensorY(sensorY));

    // Flip turning direction from phone
    int motor1Speed = scaledSensorX - scaledSensorY;
    int motor2Speed = scaledSensorX + scaledSensorY;

    // sprintln("M1: " + String(motor1Speed) + 
    //                " M2: " + String(motor2Speed) + 
    //                " A: " + String(scaledSensorY));

    setMotorSpeeds(motor1Speed, motor2Speed);
}

void bluetoothStateMachine()
{
    if (bluetooth.available() > 0)
    {
        int16_t dataRead = bluetooth.read();
        Serial.println(dataRead, HEX);

        switch (state)
        {
        case 0:
            if (dataRead == START_BYTE)
                ++state;
            else
                state = 0;
            break;

        case 1:
            receivedPacket.id = dataRead;
            ++state;
            break;
        
        case 2:
            receivedPacket.id += dataRead << 8;
            ++state;
            break;

        // TODO: Check sequence of CRC bytes from BT (Little Endian from BT)
        case 3:
            receivedPacket.crc32 = dataRead;
            ++state;
            break;

        case 4:
            receivedPacket.crc32 += dataRead << 8;
            ++state;
            break;

        case 5:
            receivedPacket.crc32 += dataRead << 16;
            ++state;
            break;

        case 6:
            receivedPacket.crc32 += dataRead << 24;
            ++state;
            break;

        case 7:
            if (dataRead == 0x1 || dataRead == 0x0)
            {
                receivedPacket.ack = (bool)dataRead;
                ++state;
            }
            else
            {
                state = 0;
            }
            break;
        
        case 8:
            receivedPacket.dataLength = dataRead;
            ++state;
            break;

        case 9:
            receivedPacket.dataLength += dataRead << 8;
            ++state;
            break;

        case 10 ... 73:
            receivedPacket.data[packetDataIndex++] = dataRead;
            ++state;
            break;

        case 74:
            if (dataRead == END_BYTE)
            {
                // TODO: Calculate data CRC32 -> if NOT same as received, throw out packet (data integrity compromised)
                uint32_t calculatedCRC32 = calculatePacketCRC32(&receivedPacket);

                if (packetCRC32Match(receivedPacket.crc32, calculatedCRC32))
                {
                    switch (receivedPacket.id)
                    {
                    case STOP_MOTORS_ID:
                        processStopMotors();
                        break;

                    case SENSOR_DATA_ID:
                        // TODO: compose uint for sensorX and sensorY from 2 bytes for each (packet data will be 4 bytes long)
                        processSensorData(getInt16FromPacketData(receivedPacket.data, 0), 
                                          getInt16FromPacketData(receivedPacket.data, 2));
                        break;

                    case PARENTAL_OVERRIDE_ID:
                        processParentalOverride();
                        break;
                    }
                }
            }

            // Reset state vars
            state = 0;

            packetDataIndex = 0;

            // TODO: Check this is needed; if not data in indexes will be overwritten
            zeroOutPacketData();

            break;
        }
    }
}

void readSerial()
{
    if (Serial.available())
    {
        while (Serial.available() > 0)
        {
            bluetooth.print(Serial.read());
        }
        bluetooth.println();
    }

    if (bluetooth.available())
    {
        while (bluetooth.available() > 0)
        {
            Serial.print(bluetooth.read(), HEX);
        }
        Serial.println('\n');
    }
}

/* ------------------------- */

uint32_t bt_delay = 500;

void setup()
{
    Serial.begin(9600);
    sprintln("Arduino Up");

    while(!Serial);

    delay(500);

    bluetooth.begin(57600);

    // delay(bt_delay);
    // bluetooth.write("AT");

    delay(bt_delay);
    bluetooth.write("AT+BAUD7");
}

void loop()
{
    safetyIsrMotors();
    bluetoothStateMachine();
    // readSerial();
}