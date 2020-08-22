/**
 * MQTT Soil Moisture Sensor
 * 
 * Monitors periodically soil moisture level and sends sensor data as string to MQTT server.
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
#include <OneWire.h>
#include <DallasTemperature.h>

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

#ifdef TEMP_SENSOR_ON
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_SENSOR_PIN);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
#endif

// Global variable keeping deep sleep setting. Might change during the programm flow
// TODO: refactor it by RUNTIME MOD enum
bool DEEP_SLEEP_enabled = false;

// todo: replace timing vars by a Timout library
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMs = 0;    // will store last time Soil Sensor was updated

// Timeout before publishing and going into sleep mode in deep sleep mode. This timout is required to be able
// to get MQTT message to switch sleep mode off
unsigned long previousDeepSleepMs = 0;   

// Sensor value
//todo: get rid of global variable
float moistureLevel = .0, temperatureC = .0;

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
  deepSleep(DEEP_SLEEP_MS);
}

/** Open Serial port for debugging purposes. */
inline void beginSerial()
{
  initSerial(SERIAL_SPEED_BAUD, SERIAL_TIMEOUT_MS);
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
  WiFi.hostname(STAHOSTNAME);

  DPRINTFF("Connecting to WiFi network ");
  DPRINTLN(STASSID);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    DPRINTLNF("Connection Failed! Restarting with delay...");
    delay(WIFI_RECONNECT_ON_FAILURE_MS);
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
      delay(CONNECT_MQTT_TIMEOUT_MS);
    }
  }
}

void publishSensorsData()
{
  //todo: get rid of global variable
  if (!client.publish(MQTT_VALUE_TOPIC, String(moistureLevel).c_str(), true)) {
    DPRINTLNF("Sending moisture level to MQTT failed");
  }

#ifdef TEMP_SENSOR_ON
  if (!client.publish(MQTT_TEMP_TOPIC, String(temperatureC).c_str(), true)) {
    DPRINTLNF("Sending temperature to MQTT failed");
  }
#endif
}

bool readSensorsData()
{
  int measurements[NUMBER_OF_MEASUREMENTS];
  float accum = .0;

  for (int i=0; i < NUMBER_OF_MEASUREMENTS; i++) {
    // read sensor raw value
    measurements[i] = analogRead(A0); // / 1023.0f;
    delay(DELAY_BETWEEN_MEASUREMENTS_MS);
  }
  
  for (int i=0; i < NUMBER_OF_MEASUREMENTS; i++) {
    accum += measurements[i];
  }
  moistureLevel = accum /NUMBER_OF_MEASUREMENTS;

  DPRINTFF("Moisture: ");
  DPRINTLN(moistureLevel);

#ifdef TEMP_SENSOR_ON
  yield();
  sensors.requestTemperatures(); 
  temperatureC = sensors.getTempCByIndex(0);

  DPRINTFF("Temp: ");
  DPRINTLN(temperatureC);
#endif

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
  DPRINTLN(STAHOSTNAME);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(STAHOSTNAME);

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
  DPRINT(F("Sketch starting: iot-moisture-sensor "));
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

#ifdef TEMP_SENSOR_ON
  // Start the DS18B20 sensor
  sensors.begin();
#endif
  powerBusOn();

  if (DEEP_SLEEP_enabled)
  {
    // Let the sensor to initialize itself
    delay(DHT_INITIALIZE_TIMEOUT_MS);
    readSensorsData();
    powerBusOff();
  }

  connectToWiFi();
  connectToMQTT();

  client.subscribe(DEEP_SLEEP_TOPIC);
  client.setCallback(callback);

  setupOTA();

  previousDeepSleepMs = millis();
}

void loop()
{
  client.loop();
  ArduinoOTA.handle();
  
  if (DEEP_SLEEP_enabled)
  {
    unsigned long currentMs = millis();
    if (currentMs - previousDeepSleepMs >= START_STATION_TIMEOUT_IN_SLEEP_MODE_MS)
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

  unsigned long currentMs = millis();
  if (currentMs - previousMs >= DHT_READ_INTERVAL_NON_SLEEP_MODE_MS)
  {
    // save the last time you updated the DHT values
    previousMs = currentMs;

    readSensorsData();
    publishSensorsData();
  }
}
