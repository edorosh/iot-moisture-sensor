/*
  Serial.h - Serial facade
*/

#include <HardwareSerial.h>

/** Starts Arduino Serial connection with the given baud and timeout in microseconds. */
void initSerial(unsigned long baud, unsigned long timeout)
{
  Serial.begin(baud);
  Serial.setTimeout(timeout);

  // Wait for serial to initialize.
  while (!Serial)
  {
  }
}