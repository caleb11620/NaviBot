#ifndef DEBUG_MACROS_H
#define DEBUG_MACROS_H

#include "Arduino.h"

// Debug mode: Uncomment/comment following line to toggle DEBUG mode
//#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(...)      Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)    Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...)     Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTF(...)
#endif

#endif // DEBUG_MACROS_H