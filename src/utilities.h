#pragma once

#include <Arduino.h>

/**
 * Utility function for printing data in hex
 * format.
 *
 * @param num the data to print in hex
 */
void printHex(long num)
{
  if (num < 0x10)
  {
    Serial.print("0");
  }
  Serial.print(num, HEX);
}

/**
 * Utility function for scaling a sensor value.
 *
 * @param value pointer to the value to scale
 * @param scale pointer to the scale amount
 * @return the scaled value as a float.
 */
float scaleSensorValue(int16_t *value, float *scale)
{
  return *value * abs(*scale);
}
