/*
  DeepSleep.h - Sends esp8266 into a deep sleep mode.
*/

#include <ESP8266WiFi.h>

void deepSleep(int microSeconds)
{
  //https://github.com/esp8266/Arduino/issues/644
  WiFi.disconnect(true);
  yield();

  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  yield();
  
  ESP.deepSleep(microSeconds * 1000);
}