#pragma once

#include <Arduino.h>

#define sprint(x) Serial.print(x)
#define sprintln(x) Serial.println(x)

void printHex(long num)
{
  if (num < 0x10) Serial.print("0");
  Serial.print(num, HEX);
}

float scaleSensorValue(int16_t* value, float* scale) 
{
    return floor(*value * abs(*scale));
}
