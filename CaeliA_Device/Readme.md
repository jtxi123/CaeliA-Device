# Note on libraries

## The following libraries have bee used:

* MH-Z19 v 1.5.3 by Jonathan Dempsey:
* WifiManager v2.03 by tzapu
* DHT Sensor Library by Adafruit v1.4.1
* Adafruit SSD1306 Library by Adafruit v2.4.1
* esp32FOTA Library by Chris Joice v 0.1.2
* Thingsboard Library by Thingsboard Team v 0.4.0
* ArduinoHttpClient Library by Arduino v 0.4.0
* Adafruit BME280 Library by Adafruit v 2.1.2
* Arduino 107 debug library by Alexander Entinger
(note:  I had to eliminate the dtostrf.c file from the library since it redefines
the dtostrf which is already defined in the core libraries of the ESP32.)

All libraries can be located through arduino library manager or through the links provided.

## A library was written to support the CM1106
* The library is included in the library folder
