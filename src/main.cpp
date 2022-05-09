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

/**
 * 
 * 
 */
void processDriveParameters(float scaleX, float scaleY)
{
    sensorXScale = scaleX;
    sensorYScale = scaleY;
}

/**
 * Method which determines what to do upon successful 
 * incoming packet processing.
 * 
 * By default, if the packet CRC is invalid, the packet is thrown out
 * and the method returns.
 */
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

/**
 * Checks after a defined interval of time whether 
 * or not the global state packet's (updated with successful incoming packet processing)
 * ACK field is set or not.
 * 
 * If ACK not set, then the motors are stopped.
 * 
 * NOTE: This method is also mutative with respect to the global state packet. When the interval
 * of time has passed, the ACK field of the global state packet is set to false in order to
 * reset the state and wait for an incoming packet to update the ACK field.
 */
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

/**
 * Passes incoming data if available to the state-machine for processing.
 * 
 * State machine processes data byte by byte. It is passed a pointer to the packet structure to
 * fill as the incoming data is processed into a potential packet; as well as a pointer
 * to a function to call upon successfully processing the incoming packet.
 * 
 * NOTE: Because the state-machine receives a pointer to a packet structure, the state-machine method
 * is mutative, and changes to the state of the packet will occur.
 */
void handleBluetooth()
{
    if (Serial1.available() > 0)
        stateMachine.loop(Serial1.read(), &receivedPacket, &processPacket);
}

/**
 * Reads and prints Serial and Serial1 data if either are available
 * 
 * Serial1 prints each data byte as hex.
 */
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

/**
 * Arduino Setup method
 * 
 * Sets up Serial and Serial1 ports, and motors.
 */
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

/**
 * Arduino Loop method
 * 
 * Continuously monitors whether to stop the motors
 * if no ACK response has been received from the phone as a safety 
 * precaution.
 * 
 * After motor safety checks, any incoming data is handled by the state-machine
 * to determine whether the data is a valid packet.
 */
void loop()
{
    handleMotorSafety();
    handleBluetooth();
    // readSerial();
}
