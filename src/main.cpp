#include "utilities.h"
#include "motors.h"
#include "arduino_packet.h"
#include "arduino_state_machine.h"

/*
Note:
On Arduino Mega I decided to put HC-06
Bluetooth module on Serial1 (pins 18 & 19)
*/

ArduinoPacket receivedPacket;
ArduinoStateMachine stateMachine;

bool stoppedMotors = false;

uint64_t lastTime = 0;
uint32_t safetyDuration = 350; /* in milliseconds */

float sensorXScale = .50f;
float sensorYScale = .115f;

// Functions

void processStopMotors()
{
    sprintln("Stopping Motors");
    setMotorSpeeds(0x0, 0x0);
}

void processParentalOverride()
{
    String prefix = "Parental Override ";
    String status = receivedPacket.data[0] ? "Active" : "Inactive";
    sprintln(prefix + status);
}

void processSensorData(int16_t sensorX, int16_t sensorY)
{
    int16_t scaledSensorX = (int16_t)scaleSensorValue(&sensorX, &sensorXScale);
    int16_t scaledSensorY = (int16_t)scaleSensorValue(&sensorY, &sensorYScale);

    // Flip turning direction from phone
    int motor1Speed = scaledSensorX - scaledSensorY;
    int motor2Speed = scaledSensorX + scaledSensorY;

    setMotorSpeeds(motor1Speed, motor2Speed);
}

void processDriveParameters(float scaleX, float scaleY)
{
    sensorXScale = scaleX;
    sensorYScale = scaleY;
}

void processPacket()
{
    if (!receivedPacket.crcOk())
    {
        sprintln("CRC Mismatch - Dismissing Packet");
        return;
    }

    switch (receivedPacket.id)
    {
    case STOP_MOTORS_ID:
        processStopMotors();
        break;

    case PARENTAL_OVERRIDE_ID:
        processParentalOverride();
        break;

    case SENSOR_DATA_ID:
        processSensorData(receivedPacket.getInt16(0),
                          receivedPacket.getInt16(2));
        break;

    case DRIVE_PARAMETERS_ID:
        processDriveParameters(receivedPacket.getFloat(0),
                               receivedPacket.getFloat(4));
        break;
    }
}

void handleMotorSafety()
{
    if (millis() < lastTime + safetyDuration)
        return;

    lastTime = millis();

    if (receivedPacket.ack)
    {
        stoppedMotors = false;
    }
    else if (!stoppedMotors)
    {
        setMotorSpeeds(0x0, 0x0);
        stoppedMotors = true;
    }

    receivedPacket.ack = false;
}

void handleBluetooth()
{
    if (Serial1.available() > 0)
        stateMachine.loop(Serial1.read(), &receivedPacket, &processPacket);
}

void readSerial()
{
    if (Serial.available())
    {
        while (Serial.available() > 0)
        {
            Serial1.print(Serial.read());
        }
        Serial1.println();
    }

    if (Serial1.available())
    {
        while (Serial1.available() > 0)
        {
            Serial.print(Serial1.read(), HEX);
        }
        Serial.println();
    }
}

void setup()
{
    Serial.begin(38400);
    sprintln("Arduino Up");

    setupMotors();

    while (!Serial);

    delay(450);

    Serial1.begin(57600);

    // delay(450);
    // Serial1.write("AT+BAUD7");
}

void loop()
{
    handleMotorSafety();
    handleBluetooth();
    // readSerial();
}
