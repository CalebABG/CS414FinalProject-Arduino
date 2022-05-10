#pragma once

#include <Arduino.h>

#define MIN_MOTOR_SPEED 0x0
#define MAX_MOTOR_SPEED 0xFF

// Motor Enable Pins
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

/**
 * Method to setup Motor pins as outputs
 * 
 */
void setupMotors()
{
    pinMode(EnA, OUTPUT);
    pinMode(EnB, OUTPUT);

    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);
    pinMode(In3, OUTPUT);
    pinMode(In4, OUTPUT);
}

/**
 * Sets the speed of the Motor.
 * 
 * @param motor the motor to set the speed
 * @param motorSpeed the speed to set
 */
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

    uint16_t constrainedSpeed = (uint16_t)constrain(abs(motorSpeed), MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    analogWrite(motorEnaPin, constrainedSpeed);
}

/**
 * Handy helper function to set both Motor speeds
 * 
 * @param leftMotorSpeed the left motor speed
 * @param rightMotorSpeed the right motor speed
 */
void setMotorSpeeds(int16_t leftMotorSpeed, int16_t rightMotorSpeed)
{
    setMotorSpeed(LEFT, leftMotorSpeed);
    setMotorSpeed(RIGHT, rightMotorSpeed);
}
