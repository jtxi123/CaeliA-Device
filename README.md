# CaeliA-Device
![](./CaeliA_Device/Images/Dispositivo%20CaeliA.jpg)

This project describes the design of a CO2 metering for deployment in schools and other public and private places.

The device takes CO2 as well as other environmental readings periodically and publishes them using mqtt to a cloud platform.
The device is equiped with a display and some indicator leds in order to also allow local operation and to signal the warning
CO2 levels.

The device was built using an ESP32 nodemcu and utilizes two sensors:
* MHZ19 CO2 sensor: 
    - An NDIR co2 sensor that supports a serial UART interface.
* DHT11 Humidity and Temperature sensor: 
    - The sensor provides temperature and humidity and is interfaced though a PWM

The device needs a wifi hotspot to connect to Internet and publish the readings.

In order to connect, the device uses the [WifiManager library](https://github.com/zhouhan0126/WIFIMANAGER-ESP32) to request from the user the connection parameters. 

* On first instalation, the device will start a captive portal AP and request the following:
    - WiFi SSID
    - WiFi password
    - MQTT broker address
    - MQTT broker port
    - MQTT topic
    - URL for to check for software updates.
* After parameters are set, the device will connect automatically without user intervention. Parameters are persisted
in the flash SPFFS file system and will be recovered in subsequent reboots.
* The user can force the device to raise the captive portal upon powerup to change any of the existing parameters.

The device uses the [fota library](https://github.com/chrisjoyce911/esp32FOTA) library for over the air updates. In will regularly check for firmware updates by checking current firmware version against the version recorded in the sw server.
In the case there is a new version it will download and install it unattended.

# Schematics of the device:

![](./CaeliA_Device/Images/Esquema.jpg)

# and the PCB

![](./CaeliA_Device/Images/CaeliA_pcb.jpg)

# MQTT Cloud platform

CaeliA device is conceived as a device used to publish into a cloud platform. The cloud platform can also be used to manage the device.

The current platform used is ThingsBoard which allows also also to issue RPC commands that the device can interpret. Curtrently the following
* RPC calls over MQTT are implemented:
    - setOffset
    - calibration
    - userMessage
    - setLedState
    - setWarnings

