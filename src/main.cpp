#include <Arduino.h>
#include <AltSoftSerial.h>
#include <CRC32.h>

// Define functions
#define Bluetooth Serial1

#define sprint(x) Serial.print(x)
#define sprintln(x) Serial.println(x)

#define START_BYTE 0x1
#define END_BYTE 0x4
#define DATA_LENGTH 0x10

// Update this start index if packet format is changed
#define DATA_START_INDEX 0xA

#define SENSOR_DATA_ID 0xD7
#define DRIVE_PARAMETERS_ID 0xD8
#define STOP_MOTORS_ID 0xE0
#define PARENTAL_OVERRIDE_ID 0xDF

#define MIN_MOTOR_SPEED 0x0
#define MAX_MOTOR_SPEED 0xFF

// H-Bridge Pins
#define EnA 5
#define EnB 6

// Motor A
#define In1 22
#define In2 23

// Motor B
#define In3 24
#define In4 25

enum Motor
{
    LEFT,
    RIGHT
};

/*
NOTES:

--------------

Number Data Types
    - https://www.arduino.cc/en/reference/int

On the Arduino Uno (and other ATMega based boards)
    int stores a 16-bit (2-byte) value.

On the Arduino Due and SAMD based boards (like MKR1000 and Zero)
    int stores a 32-bit (4-byte) value.

--------------

Arduino is Little Endian
    - https://forum.arduino.cc/t/little-endian-or-big-endian/41382

--------------

57600 baud = 7200 bytes per second
    - https://lucidar.me/en/serialib/most-used-baud-rates-table/
*/

// Note: for state-machine: range 10 - 17 is 8 numbers
typedef struct GoPacket_t
{
    byte startByte = START_BYTE; // 0
    uint16_t id; // 1 - 2
    uint32_t crc32; // 3 - 6
    bool ack; // 7
    uint16_t dataLength; // 8 - 9
    byte data[DATA_LENGTH] = {0}; // 10 - 21
    byte endByte = END_BYTE; // 22

} GoPacket;

static const GoPacket EmptyPacket;

CRC32 crc;

uint64_t timeNow = 0;
uint32_t safetyTimer = 250;
bool stoppedMotors = false;

uint32_t state = 0;

GoPacket receivedPacket;
uint16_t packetDataIndex = 0;

float sensorXScaleFactor = .50f;
float sensorYScaleFactor = .115f;

// Functions

void printHex(long num)
{
  if (num < 0x10) Serial.print("0");
  Serial.print(num, HEX);
}

// CRC32 consists of: id, ack, dataLength, data
uint32_t calculatePacketCRC32(GoPacket* packet)
{
    crc.update((byte)(packet->id)); // Low byte
    crc.update((byte)((packet->id >> 8))); // High byte

    crc.update((byte)packet->ack);

    crc.update((byte)(packet->dataLength));
    crc.update((byte)((packet->dataLength >> 8)));

    crc.update(packet->data, DATA_LENGTH);

    uint32_t calcCRC32 = crc.finalize();
    crc.reset();

    return calcCRC32;
}

bool packetCRC32Match(uint32_t receivedPacketCRC32, uint32_t calculatedPacketCRC32)
{
    return receivedPacketCRC32 == calculatedPacketCRC32;
}

/* Reference: https://stackoverflow.com/questions/3991478/building-a-32-bit-float-out-of-its-4-composite-bytes */
float getFloatFromPacketData(byte *bytes, uint16_t startIndex) {
    float f;
    byte* fPtr = (byte*)&f;

    fPtr[0] = bytes[startIndex];
    fPtr[1] = bytes[startIndex + 1];
    fPtr[2] = bytes[startIndex + 2];
    fPtr[3] = bytes[startIndex + 3];

    return f;
}

int16_t getInt16FromPacketData(byte* packetData, uint32_t startIndex)
{
    // Little Endian
    return packetData[startIndex] + 
           (((uint16_t) packetData[startIndex + 1]) << 8);
}

int32_t getInt32FromPacketData(byte* packetData, uint32_t startIndex)
{
    // Little Endian
    return packetData[startIndex] + 
           (((uint32_t) packetData[startIndex + 1]) << 8) + 
           (((uint32_t) packetData[startIndex + 2]) << 16) + 
           (((uint32_t) packetData[startIndex + 3]) << 24);
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
    return abs(sensorXScaleFactor) * v;
}

float scaleSensorY(int16_t v)
{
    return abs(sensorYScaleFactor) * v;
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
        else
        {
            if (!stoppedMotors)
            {
                // sprintln("Safety: Stopping Motors");
                setMotorSpeeds(0x0, 0x0);
                stoppedMotors = true;
            }
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
    sprintln("Parental Override Deactivated");
}

void processSensorData(int16_t sensorX, int16_t sensorY)
{
    int16_t scaledSensorX = (int16_t)floor(scaleSensorX(sensorX));
    int16_t scaledSensorY = (int16_t)floor(scaleSensorY(sensorY));

    // Flip turning direction from phone
    int motor1Speed = scaledSensorX - scaledSensorY;
    int motor2Speed = scaledSensorX + scaledSensorY;

    setMotorSpeeds(motor1Speed, motor2Speed);
}

void processDriveParameters(float sensorXScale, float sensorYScale)
{
    sensorXScaleFactor = sensorXScale;
    sensorYScaleFactor = sensorYScale;
}

void processPacket()
{
    switch (receivedPacket.id)
    {
    case STOP_MOTORS_ID:
        processStopMotors();
        break;

    case PARENTAL_OVERRIDE_ID:
        processParentalOverride();
        break;

    case SENSOR_DATA_ID:
        processSensorData(getInt16FromPacketData(receivedPacket.data, 0),
                          getInt16FromPacketData(receivedPacket.data, 2));
        break;

    case DRIVE_PARAMETERS_ID:
        processDriveParameters(getFloatFromPacketData(receivedPacket.data, 0),
                               getFloatFromPacketData(receivedPacket.data, 4));
        break;
    }
}

void bluetoothStateMachine()
{
    if (Bluetooth.available() > 0)
    {
        int16_t dataRead = Bluetooth.read();
        // printHex(dataRead); sprintln();
        // sprintln(String(state) + " - " + String(dataRead, HEX));

        switch (state)
        {
        case 0:
            if (dataRead == START_BYTE)
                state++;
            else
                state = 0;
            break;

        case 1:
            receivedPacket.id = dataRead;
            state++;
            break;
        
        case 2:
            // When shifting, need to cast to larger value to keep data
            receivedPacket.id += ((uint16_t) dataRead) << 8;
            state++;
            break;

        case 3:
            receivedPacket.crc32 = ((uint32_t) dataRead);
            state++;
            break;

        case 4:
            receivedPacket.crc32 += ((uint32_t) dataRead) << 8;
            state++;
            break;

        case 5:
            receivedPacket.crc32 += ((uint32_t) dataRead) << 16;
            state++;
            break;

        case 6:
            receivedPacket.crc32 += ((uint32_t) dataRead) << 24;
            state++;
            break;

        case 7:
            if (dataRead == 0x1 || dataRead == 0x0)
            {
                receivedPacket.ack = dataRead;
                state++;
            }
            else
            {
                state = 0;
            }
            break;
        
        case 8:
            receivedPacket.dataLength = dataRead;
            state++;
            break;

        case 9:
            receivedPacket.dataLength += ((uint16_t) dataRead) << 8;
            state++;
            break;

        case DATA_START_INDEX ... (DATA_START_INDEX + (DATA_LENGTH - 1)):
            receivedPacket.data[packetDataIndex++] = dataRead;
            state++;
            break;

        case (DATA_START_INDEX + DATA_LENGTH):
            if (dataRead == END_BYTE)
            {
                if (packetCRC32Match(receivedPacket.crc32, calculatePacketCRC32(&receivedPacket)))
                    processPacket();
            }

            state = 0;
            packetDataIndex = 0;
            receivedPacket = EmptyPacket;
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
            Bluetooth.print(Serial.read());
        }
        Bluetooth.println();
    }

    if (Bluetooth.available())
    {
        while (Bluetooth.available() > 0)
        {
            Serial.print(Bluetooth.read(), HEX);
        }
        Serial.println();
    }
}

/* ------------------------- */

uint32_t bt_delay = 500;

void setup()
{
    Serial.begin(38400);
    sprintln("Arduino Up");

    setupMotors();

    while(!Serial);

    delay(450);

    Bluetooth.begin(57600);

    // delay(bt_delay);
    // bluetooth.write("AT");

    // delay(bt_delay);
    // bluetooth.write("AT+BAUD7");
}

void loop()
{
    safetyIsrMotors();
    bluetoothStateMachine();
    // readSerial();
}