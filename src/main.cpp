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

// TODO: Arduino is Little Endian - https://forum.arduino.cc/t/little-endian-or-big-endian/41382
// TODO: 57600 baud = 7200 bytes/s
// TODO: Add back CRC, add back data array (64 bytes)
typedef struct GoPacket_t
{
    byte startByte = START_BYTE; // 0
    byte id; // 1
    uint32_t crc32; // 2 - 5
    bool ack; // 6
    byte dataLength; // 7
    byte data[DATA_LENGTH] = {0}; // 8 - 72
    byte endByte = END_BYTE; // 73

} GoPacket;

AltSoftSerial bluetooth;

uint64_t timeNow = 0;
uint32_t safetyTimer = 200;

bool receivedAck = false;
bool stoppedMotors = false;

uint32_t state = 0;

byte packetId;
uint32_t packetCRC32;
byte packetDataLength;
byte packetDataIndex = 0;
byte packetData[DATA_LENGTH] = {0};

// Functions

int getIntFromPacketData(byte* packetData, uint32_t startIndex)
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
        packetData[i] = 0;
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

void setMotorSpeed(Motor motor, int motorSpeed)
{
    int motorEnaPin,
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

    analogWrite(motorEnaPin, (int)constrain(abs(motorSpeed), MIN_MOTOR_SPEED, MAX_MOTOR_SPEED));
}

void setMotorSpeeds(int leftMotorSpeed, int rightMotorSpeed)
{
    setMotorSpeed(LEFT, leftMotorSpeed);
    setMotorSpeed(RIGHT, rightMotorSpeed);
}

float scaleSensorX(int v)
{
    return .40 * v;
}

float scaleSensorY(int v)
{
    return v * .115;
}

void safetyIsrMotors()
{
    if (millis() > timeNow + safetyTimer)
    {
        timeNow = millis();

        if (receivedAck)
        {
            stoppedMotors = false;
        }
        else if (!stoppedMotors)
        {
            // sprintln("Safety: Stopping Motors");
            setMotorSpeeds(0x0, 0x0);
            stoppedMotors = true;
        }

        receivedAck = false;
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

void processSensorData(int sensorX, int sensorY)
{
    int scaledSensorX = (int)floor(scaleSensorX(sensorX));
    int scaledSensorY = (int)floor(scaleSensorY(sensorY));

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
        int dataRead = bluetooth.read();
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
            packetId = dataRead;
            ++state;
            break;

        // TODO: Check sequence of CRC bytes from BT (Little Endian from BT)
        case 2:
            packetCRC32 = (uint32_t) dataRead << 0;
            ++state;
            break;

        case 3:
            packetCRC32 += (uint32_t) dataRead << 8;
            ++state;
            break;

        case 4:
            packetCRC32 += (uint32_t) dataRead << 16;
            ++state;
            break;

        case 5:
            packetCRC32 += (uint32_t) dataRead << 24;
            ++state;
            break;

        case 6:
            if (dataRead == 0x1 || dataRead == 0x0)
            {
                receivedAck = (bool)dataRead;
                ++state;
            }
            else
            {
                state = 0;
            }
            break;
        
        case 7:
            packetDataLength = dataRead;
            ++state;
            break;

        case 8 ... 72:
            packetData[packetDataIndex++] = dataRead;
            ++state;
            break;

        case 73:
            if (dataRead == END_BYTE)
            {
                // TODO: Calculate data CRC16 -> if NOT same as received, throw out packet (data integrity compromised)
                switch (packetId)
                {
                case STOP_MOTORS_ID:
                    processStopMotors();
                    break;

                case SENSOR_DATA_ID:
                    // TODO: compose uint for sensorX and sensorY from 2 bytes for each (packet data will be 4 bytes long)
                    processSensorData(getIntFromPacketData(packetData, 0), getIntFromPacketData(packetData, 4));
                    break;

                case PARENTAL_OVERRIDE_ID:
                    processParentalOverride();
                    break;
                }
            }

            // Reset state vars
            state = 0;

            packetId = 0;
            packetCRC32 = 0;
            packetDataIndex = 0;
            packetDataLength = 0;
            zeroOutPacketData();

            break;

        default:
            ++state;
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