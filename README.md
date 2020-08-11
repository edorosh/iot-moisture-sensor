# MQTT Soil Moisure Sensor
Monitors periodically soil moisture level and sends sensor data as string to MQTT server. Tested on NodeMCU V3 and Wemos D1 mini.
ESP8266 chip enters sleep mode to save extra eneregy for a battery powered devices.

# Circuit
TODO

# Configuration
Please copy file `Config.h.local` into `Config.h` and define your MQTT and Network credentials there before building the firmware. The firmware expectsusing static IP address to save a battery energy. OTA is not implemented on purpose since it doesn't work out of the box in a deep sleep mode.

> Make sure not to commit your credentials to Git. `Config.h` is ignored in file .gitignore.

# TODO
* Enable local AP mode to change WiFi creds without firmwaring it