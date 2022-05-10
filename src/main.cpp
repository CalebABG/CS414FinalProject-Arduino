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

uint64_t lastTime = 0;         /* Timestamp in milliseconds */
uint32_t safetyDuration = 350; /* Duration of time in milliseconds */

/* Drive Parameters (scales forward and backward speed, and turning speed) */
float sensorXScale = .50f;
float sensorYScale = .115f;

/**
 * Method which handles stopping the motors if stopping the motors was
 * requested from the phone.
 */
void processStopMotors()
{
    Serial.println("Stopping Motors");
    setMotorSpeeds(0x0, 0x0);
}

/**
 * Method which handles whether parental override
 * is active or inactive.
 *
 * Currently method only prints the status, but can be extended
 * to do other work if needed.
 */
void processParentalOverride()
{
    String prefix = "Parental Override ";
    String status = receivedPacket.data[0] ? "Active" : "Inactive";
    Serial.println(prefix + status);
}

/**
 * Method which sets the motors speed based on the
 * drive parameters and sensorX and sensorY values.
 *
 * @param sensorX Forward / Backward speed
 * @param sensorY Turning speed
 */
void processSensorData(int16_t sensorX, int16_t sensorY)
{
    int16_t scaledSensorX = (int16_t)floor(scaleSensorValue(&sensorX, &sensorXScale));
    int16_t scaledSensorY = (int16_t)floor(scaleSensorValue(&sensorY, &sensorYScale));

    // Attempt at addressing motor deadband
    // Actual correction requires getting better motors w/ encoders ¯\_(ツ)_/¯
    int16_t deadBandAdjustment = 1;
    int16_t deadBandCorrection = sensorX > 0 ? deadBandAdjustment : -deadBandAdjustment;

    // Switch the '+' or '-' on motor speed calculations to change
    // turning direction from phone
    int16_t motor1Speed = scaledSensorX - scaledSensorY;
    int16_t motor2Speed = (scaledSensorX + deadBandCorrection) + scaledSensorY;

    setMotorSpeeds(motor1Speed, motor2Speed);
}

/**
 * Method which sets the drive parameters.
 */
void processDriveParameters(float scaleX, float scaleY)
{
    sensorXScale = scaleX;
    sensorYScale = scaleY;
}

/**
 * Method which determines what to do upon successful
 * processing of an incoming packet.
 *
 * By default, if the packet CRC is invalid, the packet is thrown out
 * and the method returns.
 */
void processPacket()
{
    if (!receivedPacket.crcOk())
    {
        Serial.println("CRC Mismatch - Dismissing Packet");
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
 * Method which sends an information packet over
 * Bluetooth.
 *
 * Can add status indicators, other sensor data, etc...
 * to the packets data array. Increase the DATA_LENGTH
 * define if theres a need/want to send more data in the packet.
 */
void sendArduinoInfo()
{
    ArduinoPacket packet;
    packet.id = ARDUINO_INFO_ID;
    packet.ack = false;

    /*
    Format of data from start of data to end:
    - Parental Control Status
    */
    packet.data[0] = receivedPacket.ack;

    packet.crc = packet.calculateCRC();

    /* NOTE:
    This sends more bytes than the number of bytes of each member field in the struct
    due to alignment padding - https://www.geeksforgeeks.org/is-sizeof-for-a-struct-equal-to-the-sum-of-sizeof-of-each-member/
    */
    Serial1.write((uint8_t *)&packet, sizeof(packet));
}

/**
 * Checks after an interval of time whether
 * or not the global state packet's ACK field is set or not.
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
    {
        return;
    }

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
    {
        stateMachine.loop(Serial1.read(), &receivedPacket, &processPacket);
    }
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
    Serial.println("Arduino Up");

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
