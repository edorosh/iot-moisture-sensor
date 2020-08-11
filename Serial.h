/*
  Serial.h - Serial facade
*/

#ifndef Serial_h
#define Serial_h

/** Sends esp8266 into a deep sleep mode for the given micro seconds. */
void initSerial(unsigned long baud, unsigned long timeout);

#endif