/**
 * MQTT Soil Moisture Sensor
 * 
 * Monitors periodically soil moisture level and sends JSON data to MQTT server. Tested on NodeMCU V3.
 * ESP8266 chip enters sleep mode to save extra eneregy for a battery powered devices.
 * 
 * Author: Evgeny Doros
 * Email: eugene.dorosh@gmail.com
 * Github: https://github.com/edorosh
 * License: MIT 
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

#include "DeepSleep.h"
#include "Serial.h"

#include "Config.h"
#include "DebugMacro.h"
#include "Version.h"

// define the number of bytes we need to save deep sleep state
#define EEPROM_SIZE 1

WiFiClient espClient;
PubSubClient client(espClient);

// static IP address of device
IPAddress IPADDRESS_CONFIG;  
IPAddress GATEWAY_CONFIG;
IPAddress SUBNET_CONFIG; 

// Global variable keeping deep sleep setting. Might change during the programm flow
bool DEEP_SLEEP_enabled = false;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;    // will store last time Soil Sensor was updated

// Timeout before publishing and going into sleep mode in deep sleep mode. This timout is required to be able
// to get MQTT message to switch sleep mode off
unsigned long previousDSMillis = 0;   
const long intervalDS = 1e3; 

// Sensor value
float moistureLevel = .0;

/**
 * Closes all network clients and sends the chip into Deep Sleep Mode with WAKE_RF_DEFAULT.
 * The function does nothing in case global variable DEEP_SLEEP_enabled is true.
 */
void enterDeepSleepMode()
{
  if (client.connected())
  {
    DPRINTLNF("Disconnecting MQTT Client");
    client.disconnect();
  }

  DPRINTLNF("Entering a deep sleep mode");
  deepSleep(DEEP_SLEEP_MICRO_SECONDS);
}

/** Open Serial port for debugging purposes. */
inline void beginSerial()
{
  initSerial(SERIAL_SPEED_BAUD, SERIAL_TIMEOUT_MICRO_SECONDS);
  Serial.println(F("Booting..."));
}

inline void connectToWiFi()
{
  WiFi.forceSleepWake();
  yield();

  DPRINTLNF("Enabling STA mode");

  // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
  WiFi.persistent(false);

  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  WiFi.config(ip, gateway, subnet);

  DPRINTFF("Connecting to WiFi network ");
  DPRINTLN(STASSID);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    DPRINTLNF("Connection Failed! Restarting in 500ms...");
    delay(500);
    ESP.restart();
  }

  DPRINTFF("Connected! IP address is ");
  DPRINTLN(WiFi.localIP().toString().c_str());
}

void powerBusOn()
{
  DPRINTLNF("Set Power Bus ON by pin to HIGH");

  digitalWrite(DHT_KEY_PIN, HIGH);
  yield();
}

void powerBusOff()
{
  DPRINTLNF("Set DHT OFF by pin to LOW");

  digitalWrite(DHT_KEY_PIN, LOW);
  yield();
}

inline void connectToMQTT()
{
  client.setServer(MQTTSERVER, MQTTPORT);

  // Loop until we're reconnected
  while (!client.connected())
  {
    DPRINT("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect(MQTT_CLIENT_NAME))
    {
      DPRINTLNF(" connected");
    }
    else
    {
      DPRINTFF(" failed, rc=");
      DPRINT(client.state());
      DPRINTLNF(" try again in 5 seconds");

      // Wait some time before retrying
      delay(CONNECT_MQTT_TIMEOUT_MICRO_SECONDS);
    }
  }
}

void publishSensorsData()
{
  if (!client.publish(MQTT_VALUE_TOPIC, String(moistureLevel).c_str(), true)) {
    DPRINTLNF("Sending message to MQTT failed");
  }
}

bool readSensorsData()
{
   // read sensor raw value
  moistureLevel = analogRead(A0); // / 1023.0f;

  DPRINTFF("Moisture: ");
  DPRINTLN(moistureLevel);

  return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  DPRINTFF("Message arrived [");
  DPRINT(topic);
  DPRINTFF("] ");
  for (int i = 0; i < length; i++) {
    DPRINT((char)payload[i]);
  }
  DPRINTLN();

  // Backup old value
  bool last_DEEP_SLEEP_enabled = DEEP_SLEEP_enabled;

  // Switch on Deep Sleep if an 1 was received as first character
  DEEP_SLEEP_enabled = ((char)payload[0] == '1');
  
  // DEBUG
  if (DEEP_SLEEP_enabled) {
    DPRINTLNF("Deep Sleep enabled");
  } else {
    DPRINTLNF("Deep Sleep disabled");
  }

  // save the DEEP_SLEEP state in flash memory
  if (last_DEEP_SLEEP_enabled != DEEP_SLEEP_enabled) {
    DPRINTLNF("Writing to EEPROM deep sleep mode");
    EEPROM.write(0, DEEP_SLEEP_enabled);
    EEPROM.commit();
    
    DPRINTLNF("Restarting ESP ...");
    ESP.restart();
  }
}

inline void setupOTA()
{
  DPRINT("Enabling OTA with hostname ");
  DPRINTLN(OTAHOSTNAME);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OTAHOSTNAME);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    DPRINTLN("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    DPRINTLN("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DPRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    DPRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      DPRINTLNF("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      DPRINTLNF("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      DPRINTLNF("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      DPRINTLNF("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      DPRINTLNF("End Failed");
    }
  });

  ArduinoOTA.begin();
}

void setup()
{
#ifdef DEBUG
  beginSerial();
  DPRINTLN("");
  DPRINT(F("Sketch starting: iot-weather-station "));
  DPRINTLN(FW_VERSION);
  DPRINT(F("Reset reason: "));
  DPRINTLN(ESP.getResetReason());
  DPRINT(F("Core Version: "));
  DPRINTLN(ESP.getCoreVersion());
  DPRINT(F("SDK Version: "));
  DPRINTLN(ESP.getSdkVersion());
  DPRINTLN("");
#endif

  pinMode(DHT_KEY_PIN, OUTPUT);

  // https://www.bakke.online/index.php/2017/05/21/reducing-wifi-power-consumption-on-esp8266-part-2/
  // Turn Wifi off until we have something to send
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  yield();

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  DEEP_SLEEP_enabled = EEPROM.read(0);

  powerBusOn();

  if (DEEP_SLEEP_enabled)
  {
    // Let the sensor to initialize itself
    delay(DHT_INITIALIZE_TIMEOUT_MICRO_SECONDS);
    readSensorsData();
    powerBusOff();
  }

  connectToWiFi();
  connectToMQTT();

  client.subscribe(DEEP_SLEEP_TOPIC);
  client.setCallback(callback);

  setupOTA();

  previousDSMillis = millis();
}

void loop()
{
  client.loop();
  
  if (DEEP_SLEEP_enabled)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousDSMillis >= START_STATION_TIMEOUT_IN_SLEEP_MODE)
    {
      publishSensorsData();
      enterDeepSleepMode();
    }
    else
    {
      //DPRINTLNF("SKIP Deep Sleep Loop");
      yield();
      return;
    }
  }

  ArduinoOTA.handle();
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= DHT_READ_INTERVAL_NON_SLEEP_MODE)
  {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

    readSensorsData();
    publishSensorsData();
  }
}
