#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
//#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#define RIVAS
#define WIFI_TELEMETRY
//#define LP
#include <107-Arduino-Debug.hpp>
DEBUG_INSTANCE(200, Serial);

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>  // ThingsBoard SDK
#include <NTPClient.h>
#include <WiFiUdp.h>


// We do theoretically have an Arduino_ESP32_Updater component, which has dependencies on Arduino and used the UpdaterClass,
// but it makes not sense to use that component atleast currently, because it simply implements writing to partitions
// in a very suboptimal way, allocatng 4KB on the heap and even causing undefined behaviour and even memory leaks.
// See https://github.com/espressif/arduino-esp32/issues/7984#issuecomment-1717151822 for more information on the issues with the UpdaterClass.
// Therefore instead it is recommended to use the Epsressif_Updater which directly uses the headers, which are included in the UpdaterClass anyway,
// but because it directly use the OTA Update API see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html for more information,
// it is more efficient and does not have any of the aforementioned issues.
// Thanks to those issues it is highly recommended to use the Espressif_Updater as long as the issue has not been resolved on a Arduino level, additional the same is the case
// if an older version of Arduino is used that might not contain the possible fixes yet.
#include <Espressif_Updater.h>



// Whether the given script is using encryption or not,
// generally recommended as it increases security (communication with the server is not in clear text anymore),
// it does come with an overhead tough as having an encrypted session requires a lot of memory,
// which might not be avaialable on lower end devices.
#define ENCRYPTED false

// Firmware title and version used to compare with remote version, to check if an update is needed.
// Title needs to be the same and version needs to be different --> downgrading is possible

const bool check_signature = false;
const bool disable_security = true;
constexpr char CURRENT_FIRMWARE_TITLE[] = "CaeliA_esp32_co2";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0.1";
constexpr char FORCEUPDATE[]="9.9.9";


// Maximum amount of retries we attempt to download each firmware chunck over MQTT
//constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 16U;

// Size of each firmware chunck downloaded over MQTT,
// increased packet size, might increase download speed

constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;
//constexpr uint16_t FIRMWARE_PACKET_SIZE = 32768U;


//#define MINID1
#define DEVKIT
#include "esp32-hal-cpu.h"
// File system
#include <FS.h>
#include <SPIFFS.h>

// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port,
// whereas 8883 would be the default encrypted SSL MQTT port

#if ENCRYPTED
constexpr uint16_t THINGSBOARD_PORT = 8883U;
// See https://comodosslstore.com/resources/what-is-a-root-ca-certificate-and-how-do-i-download-it/
// on how to get the root certificate of the server we want to communicate with,
// this is needed to establish a secure connection and changes depending on the website.
#include "root_ca.h"
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;


WiFiUDP ntpUDP;
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
//
//tiempo UTC para tener hora local hy que modificar el
// offset (0) con la función timeClient.setTimeOffset(long timeOffset)
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 60000);
//NTPClient timeClient(ntpUDP);

//needed for library for autoconnect
//#include <DNSServer.h>
#include <WebServer.h>
//Debug level for WiFiManager
#define WM_DEBUG_LEVEL 1
#include <WiFiManager.h>

// These values are only relevant for the first run. After that the default values are
// those persisted in the config file.
// If the sw_server is empty the sw will skip updates.
// #define SW_SERVER "raw.githubusercontent.com"
// The json file contains the version available and the pointer to where the bin file is located
// The sw will check the version compare it with the version in the actual sw and update if necessary.
//#define SW_FILE "/jtxi123/CaeliA-Rivas/main/FotaRivas/fota.json"

// This levels can be changed using the RPC call
#define WARN 700;
#define DANGER 1000;

// Variables to store credentials in case we need to reconnect
//char ap_ssid[40];  //to store the wifi ap name
//char ap_pass[40];  //to store the wifi ap password
// directive foor CO2 sensors
// The SW will check which sensor is present. If the definition is commented
// The SW will not check for its presence
#define MHZ19_CO2
#define CM1106_CO2
// If RPC is commented the device will not support RPC
#define RPC
// device has SSD1306 128x64 display
// If DISPLAY is defined the device will check for the presence of a display
// and will use it if present.
#define DISPLAY

#define BAUDRATE 9600  // Device to CO2 sensor Serial baudrate (should not be changed)
#define RX_PIN 19      // Rx pin which the CO2 Sensor Tx pin is attached
#define TX_PIN 18      // Tx pin which the CO2 Sensor Rx pin is attached

#ifdef MHZ19_CO2
#include "MHZ19.h"
#define RANGE_Z19 5000  // Range for MHZ19 CO2 readings
MHZ19 myMHZ19;          // Constructor for library
#endif

#ifdef CM1106_CO2
#include "CM1106.h"  //Using CM1106 Sensor
CM1106 myCM1106;     //Constructor for library
#endif

typedef enum SensorType {
  NOSENSOR = 0,     //No CO2 sensor
  MHZ19sensor = 1,  // MHZ19 sensor
  CM1106sensor = 2  // CM1106 sensor
} SensorType;
SensorType co2Sensor = NOSENSOR;

// Just as with the CO2 sensor both types of temperature and humidity sensors
// can be defined. The SW will check which one is present.
#define DHT  //DHT temperature and hunidity sensor
#ifdef DHT
#include "DHTesp.h"
DHTesp dht;
// Comfort profile
ComfortState cf;
// Pin number for DHT11 data pin
int dhtPin = 23;
int dhtSensor = 0;
#endif
// BME280 temperature, humidity and pressure
#define BME280
#ifdef BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C
int bmeSensor = 0;
#endif
#define DEFAULT_TEMP_OFFSET "-3.5"


#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY !)
#error Select ESP32 board.
#endif
// Wire library to support I2C bus
#include <Wire.h>
#define I2C_SDA 5
#define I2C_SCL 4

#ifdef DISPLAY
#include <Adafruit_SSD1306.h>
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//#define I2C_SDA 21
//#define I2C_SCL 22
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin # (or -1 if sharing Arduino reset pin)

//Create the display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Bitmap Icons for display
#define wifi1_icon16x16_width 16
#define wifi1_icon16x16_height 16
//variable to register the detection of a display at address 0x3C
bool displayPresent = false;  // Variable to annotate if disply was detected
const unsigned char wifi_icon16x2[][2] = {
  0b11111111, 0b11111111,  // ################
  0b00000000, 0b00000000,  //
  0b01111111, 0b11111110,  //  ##############
  0b00000000, 0b00000000,  //
  0b00111111, 0b11111100,  //   ############
  0b00000000, 0b00000000,  //
  0b00011111, 0b11111000,  //    ##########
  0b00000000, 0b00000000,  //
  0b00001111, 0b11110000,  //     ########
  0b00000000, 0b00000000,  //
  0b00000111, 0b11100000,  //      ######
  0b00000000, 0b00000000,  //
  0b00000011, 0b11000000,  //       ####
  0b00000000, 0b00000000,  //
  0b00000001, 0b10000000,  //        ##
  0b00000000, 0b00000000,  //
};

#define arrow_up_icon16x16_width 16
#define arrow_up_icon16x16_height 16
const unsigned char arrow_up_icon16x16[] = {
  0b00000001, 0b10000000,  //        ##
  0b00000011, 0b11000000,  //       ####
  0b00000111, 0b11100000,  //      ######
  0b00001111, 0b11110000,  //     ########
  0b00011110, 0b01111000,  //    ####  ####
  0b00111100, 0b00111100,  //   ####    ####
  0b01111000, 0b00011110,  //  ####      ####
  0b11111100, 0b00111111,  // ######    ######
  0b11111100, 0b00111111,  // ######    ######
  0b01111100, 0b00111110,  //  #####    #####
  0b00011100, 0b00111000,  //    ###    ###
  0b00011100, 0b00111000,  //    ###    ###
  0b00011100, 0b00111000,  //    ###    ###
  0b00011111, 0b11111000,  //    ##########
  0b00011111, 0b11111000,  //    ##########
  0b00001111, 0b11110000,  //     ########
};

#define cancel_icon16x16_width 16
#define cancel_icon16x16_height 16
const unsigned char cancel_icon16x16[] = {
  0b00000000, 0b00000000,  //
  0b00000000, 0b00000000,  //
  0b00111000, 0b00001110,  //   ###       ###
  0b00111100, 0b00011110,  //   ####     ####
  0b00111110, 0b00111110,  //   #####   #####
  0b00011111, 0b01111100,  //    ##### #####
  0b00001111, 0b11111000,  //     #########
  0b00000111, 0b11110000,  //      #######
  0b00000011, 0b11100000,  //       #####
  0b00000111, 0b11110000,  //      #######
  0b00001111, 0b11111000,  //     #########
  0b00011111, 0b01111100,  //    ##### #####
  0b00111110, 0b00111110,  //   #####   #####
  0b00111100, 0b00011110,  //   ####     ####
  0b00111000, 0b00001110,  //   ###       ###
  0b00000000, 0b00000000,  //
};
#endif



// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

char TOKEN[40] = "CaeliA-28092EA8CC84";
char WIFI_SSID[40] = "MIWIFI_2G_Zbiz";
char WIFI_PASSWORD[40] = "ZGHhi9qh";
// ThingsBoard server instance.
//#define THINGSBOARD_SERVER "thingsboard.cloud"
#define THINGSBOARD_SERVER "demo.thingsboard.io"
//#define THINGSBOARD_SERVER  "192.168.1.100"
#define TELEMETRY_TOPIC "v1/devices/me/telemetry"

// Baud rate for debug serial console
#define SERIAL_DEBUG_BAUD 115200

#define MAX_KEYS 20      //Maximum number of Telemetry key-value pairs
#define NO_KEY ""        //empty key tag name
#define DISPLAY_LINES 8  //Number of lines on display


// Create the ThingsBoard client
#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
// Initialize ThingsBoard instance
Arduino_MQTT_Client mqttClient(espClient);

ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

//ThingsBoardSized tb(mqttClient, MAX_MESSAGE_SIZE);
//ThingsBoard tb(espClient, MAX_MESSAGE_SIZE);
// Initalize the Updater client instance used to flash binary to flash memory

Espressif_Updater updater;

// Statuses for updating
bool currentFWSent = false;
bool updateRequestSent = false;
bool updating = false;
bool forceupdateflag = false;


// the Wifi radio's status
int wifiStatus = WL_IDLE_STATUS;

DynamicJsonDocument configJson(1024);  //json document to store config parameters

//Temperature Offset
float Toffset = 0;

//Pins to control the LEDs
#ifdef MINID1
#define RED 17
#define YELLOW 21
#define GREEN 25
#else
#define RED 15
#define YELLOW 2
#define GREEN 0
#endif

// Array with LEDs that should be controlled from ThingsBoard
uint8_t leds_control[] = { RED, YELLOW, GREEN };

#ifdef MINID1
#define PIN_CONFIGURE 25  //Pin used to force configuration at boot
#else
#define PIN_CONFIGURE 3  //Pin used to force configuration at boot
#endif

#define PIN_BOOT 0
// global timeStamp variables
unsigned long current_time;
unsigned long last_time;
unsigned long display_time;
unsigned long start_time;  //record the initial millis() to wait for sensor stabilization
unsigned long bootTimeStamp;

struct CONNECTION {
  bool wifi = false;
  int wifiStrength = 0;
  bool mqtt = false;
  bool rpc = false;
} connStatus;


#ifdef RPC
// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;

// Watchdog if the system is a long time with no connection it will reboot
#define TWD 3600  // 1 hour
//#define TWD 1000 // 1000 sec
//#define TWD 0 // Disable watchdog
const unsigned long wdtTimeout = 1000 * TWD;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;
// In the case of watchdog interrupt, this function is called
void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");  // When called it reboots
  esp_restart();
}
// Take measurements of the Wi-Fi strength and return the average result.
//int fake = 0;
int getStrength(bool raw = false) {
  int rssi = 0;
  int strength = 0;
  rssi = WiFi.RSSI();
  if (!raw) {
    if (rssi > -55) strength = 7;
    else if (rssi > -59) strength = 6;
    else if (rssi > -63) strength = 5;
    else if (rssi > -67) strength = 4;
    else if (rssi > -71) strength = 3;
    else if (rssi > -75) strength = 2;
    else if (rssi > -79) strength = 1;
    //strength = fake++ % 8;
    DBG_INFO("WiFi level: %i", strength);
  } else strength = rssi;
  return strength;
}



// Processes functions for RPC calls
// RPC_Data is a JSON variant, that can be queried using operator[]
// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
// Function to modify offset to adjust co2 measurments.
int co2Offset = 0;
static StaticJsonDocument<JSON_OBJECT_SIZE(1)> docresponse;
static RPC_Response processSetOffset(const RPC_Data &data) {
  // Process data
  docresponse.clear();
  co2Offset += int(data);  //Change co2 offset

  DBG_INFO("Received the set offset RPC method");
  DBG_INFO("Set new CO2_offset: %i", co2Offset);
  docresponse["co2Offset"] = co2Offset;
  //return RPC_Response(NULL, co2Offset);
  return RPC_Response(docresponse);
}

// Function to turn on or off auto calibration.
static RPC_Response processAutoCalibration(const RPC_Data &data) {
  docresponse.clear();
  DBG_INFO("Set auto calibration");
  configJson["auto_calibration"] = bool(data);
  saveConfigFile(configJson);
#ifdef MHZ19_CO2
  if (co2Sensor == MHZ19sensor) {

    myMHZ19.autoCalibration(bool(configJson["auto_calibration"]));
    // set Range to RANGE_Z19 using a function, see below (disabled as part of calibration)
    myMHZ19.setRange(RANGE_Z19);
  }
#endif
#ifdef CM1106_CO2
  if (co2Sensor == CM1106sensor) {
    myCM1106.autoCalibration(bool(configJson["auto_calibration"]), 7);
  }
#endif
  docresponse["AutoCalibration"] = bool(configJson["auto_calibration"]);
  return RPC_Response(docresponse);
  //return RPC_Response("AutoCalibration", bool(configJson["auto_calibration"]));
}
// This flag will be raised if calibration is requested.
// it will be reset after calibration
bool calibrationFlag;
static RPC_Response processCalibration(const RPC_Data &data) {
  calibrationFlag = true;
  docresponse.clear();
  DBG_INFO("Received the calibration RPC command");
  DBG_INFO("Set the calibration flag: %i", calibrationFlag);
  docresponse["calibration"] = calibrationFlag;
  return RPC_Response(docresponse);
  //return RPC_Response("calibration", calibrationFlag);
}

// Function to receive a message with a severity level.
String userMessage = "";
int messageLevel = 0;
static RPC_Response processUserMessage(const RPC_Data &data) {
  const char *message;

  DBG_INFO("Received the get userMessage method");

  message = data["message"];
  userMessage = String(message);
  messageLevel = data["level"];

  DBG_INFO("Mensaje: %s Nivel: %i", message, messageLevel);
  docresponse["message"] = message;
  return RPC_Response(docresponse);
  //return RPC_Response("message", message);
}

// Function to control leds
// when led_state ==1 led will light in accordance with the CO2 level
// when led_state ==2 led is always on
// when led_state ==0 the led is always off

int led_state[3] = { 1, 1, 1 };

static RPC_Response processSetLedState(const RPC_Data &data) {
  docresponse.clear();
  Serial.println("Received the set setLedState RPC method");
  serializeJson(data, Serial);
  int led = data["led"];
  int enabled = data["enabled"];

  if (led < COUNT_OF(leds_control)) {

    DBG_INFO("Setting LED %i  (GPIO%i) to state %i", led, leds_control[led], enabled);
    led_state[led] = enabled;
    if (enabled == 2) digitalWrite(leds_control[led], 1);
    if (enabled == 0) digitalWrite(leds_control[led], 0);
  }
  //return RPC_Response(data["led"], enabled);
  docresponse["enabled"] = enabled;
  return RPC_Response(docresponse); 
  //return RPC_Response("enabled", enabled);
}

int level_warn = WARN;
int level_danger = DANGER;

static RPC_Response processSetWarnings(const RPC_Data &data) {
  docresponse.clear();
  Serial.println("Received the set setWarnings RPC method");

  level_warn = data["warning"];
  level_danger = data["danger"];

  DBG_INFO("Setting Warning levels");
  DBG_INFO("From 0 to %i ppm good", level_warn);
  DBG_INFO("From %i to %i ppm fair", level_warn + 1, level_danger);
  DBG_INFO("Above %i  ppm danger", level_danger);

  docresponse["warning"] = level_warn;
  return RPC_Response(docresponse); 
  //return RPC_Response("warning", level_warn);
}
static RPC_Response processUpdateFirmware(const RPC_Data &data) {
  docresponse.clear();
  DBG_INFO("Received the set updateFirmware RPC method");
  forceupdateflag=true;
  /*
  tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, FORCEUPDATE);
  static const OTA_Update_Callback forcecallback(&progressCallback, &updatedCallback, CURRENT_FIRMWARE_TITLE,
                    FORCEUPDATE, &updater, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
  DBG_INFO("Declared callback");
  // See https://thingsboard.io/docs/user-guide/ota-updates/
  // to understand how to create a new OTA pacakge and assign it to a device so it can download it.
  updateRequestSent = tb.Start_Firmware_Update(forcecallback);
  */
  DBG_INFO("Requested update");
  docresponse["Update_status"] = "Updating requested";
  return RPC_Response(docresponse);
  //return RPC_Response("update", false);
}
static RPC_Response processUpdateParameters(const RPC_Data &data) {
  docresponse.clear();
  DBG_INFO("Received the set updateParameters RPC method");
  // update only the parametes included in the call
  JsonObjectConst parameters = data;
  for (auto p : parameters) {
    DBG_INFO("key: %s value: %4.1f", p.key().c_str(), float(p.value()));
    configJson[p.key()] = p.value();
  }
  saveConfigFile(configJson);  //save the configuration file
  esp_restart();               //reboot

  //This line will only be excuted if the firmware update fails
  docresponse["parameters"] = false;
  return RPC_Response(docresponse);
  //return RPC_Response("parameters", false);
}

static RPC_Response processStatus(const RPC_Data &data) {

  static char status[45];
  docresponse.clear();

  DBG_INFO("Received the status RPC command");

  if (data) {
    docresponse[(const char *)data] = (const char *)configJson[(const char *)data];
    //return RPC_Response((const char *)data, (const char *)configJson[(const char *)data])
  }
  else {
    snprintf(status, sizeof(status), "Uptime: %us, Ver: %s", (millis() - start_time) / 1000, CURRENT_FIRMWARE_VERSION);
    DBG_INFO(status);
    // Just an response example
    // static StaticJsonDocument<JSON_OBJECT_SIZE(1)> doc;
    docresponse["status"] = status;

  }
  //serializeJson(docresponse, Serial);
  return RPC_Response(docresponse);
  //return RPC_Response((const char *)"status", status);
}
// RPC handlers for RPC

static const std::vector<RPC_Callback> callbacks = {
  { "setOffset", processSetOffset },
  { "calibration", processCalibration },
  { "autoCalibration", processAutoCalibration },
  { "userMessage", processUserMessage },
  { "setLedState", processSetLedState },
  //{"setWarnings",       processSetWarnings },
  { "updateFirmware", processUpdateFirmware },
  { "updateParameters", processUpdateParameters },
  { "status", processStatus }
};
#endif

// This measurments period MEAS_PERIOD in seconds
#define MEAS_PERIOD 20
// This is the period the disply remains ON in seconds
#define WAKE_PERIOD 600
#define WARMUPTIME 0

//  Task to reads & precess measurements from sensors

//Telemetry Json document
StaticJsonDocument<128U> telemetry;
void measTask() {
  DBG_INFO("measTask started");
  if (ESP.getFreeHeap() < 10000) esp_restart();  //There are some memory leaks
  DBG_INFO("Memoria libre: %u", ESP.getFreeHeap());
  telemetry.clear();  //clear the telemetry JsonDocument
  readMeasurement(telemetry);
  publishMeasurement(telemetry);
  displayMeasurement(telemetry);
  showLeds(telemetry);
  calibration();
  DBG_INFO("measTask ended");
}

void calibration()  //Calibrate if needed
{
  if (calibrationFlag) {
#ifdef MHZ19_CO2
    if (co2Sensor == MHZ19sensor) {
      //Check if a calibration is needed
      myMHZ19.calibrate();
      myMHZ19.autoCalibration(bool(configJson["auto_calibration"]));
      // set Range to RANGE_Z19 using a function, see below (disabled as part of calibration)
      myMHZ19.setRange(RANGE_Z19);
    }
#endif
#ifdef CM1106_CO2
    if (co2Sensor == CM1106sensor) {
      //Check if a calibration is needed
      myCM1106.calibrate();
      myCM1106.autoCalibration(bool(configJson["auto_calibration"]), 7);
    }
#endif
    display.setTextSize(2);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrate");
    display.println("CO2 sensor");
    display.setTextSize(1);
    display.display();
    //reset offset and flag
    co2Offset = 0;
    calibrationFlag = false;
  }
}


//  Reads data from sensors
int last_co2 = 0;
int delta_co2 = 10;
int last_humidity = 0;
int delta_h = 1;
float last_pressure = 0;
float delta_p = 10;
float last_temperature = 0;
float delta_t = 0.5;
int wifi_db;
int last_wifi = -99;
int delta_w = 2;
int co2_ppm;  //concentration of CO2
float temperature;
int humidity;
float pressure;
float co2_t;  //Temperature of the sensor (not very accurate as sensor generates heat)

void readMeasurement(JsonDocument &telemetry) {
  timeClient.update();
  unsigned long long ts;
  char ts_str[20];
  JsonObject tm = telemetry.to<JsonObject>();
  ts = timeClient.getEpochTime();
  ts = ts * 1000;  // in ms? UTC
  // we have to convert the ts to a string because the Json serialize arduino
  // function will not handle long long types
  sprintf(ts_str, "%llu", ts);
  tm["ts"] = ts_str;
  JsonObject values = tm.createNestedObject("values");
#ifdef MHZ19_CO2
  if (co2Sensor == MHZ19sensor) {
    // note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even
    // if below background CO2 levels or above range (useful to validate sensor). You can use the
    // usual documented command with getCO2(false)

    co2_ppm = co2Offset + myMHZ19.getCO2();  // Request CO2 (as ppm) unlimimted value, new request
    co2_t = myMHZ19.getTemperature(true);    // decimal value, not new request
    //co2_t = myMHZ19.getTempAdjustment();   // adjusted temperature

    if (abs(co2_ppm - last_co2) > delta_co2) {
      values["co2_ppm"] = co2_ppm;
      last_co2 = co2_ppm;
    }
  }
#endif
#ifdef CM1106_CO2
  if (co2Sensor == CM1106sensor) {
    co2_ppm = co2Offset + myCM1106.getCO2();  // Request CO2 (as ppm) unlimimted value, new request
    if (abs(co2_ppm - last_co2) > delta_co2) {
      values["co2_ppm"] = co2_ppm;
      last_co2 = co2_ppm;
    }
  }
#endif

#ifdef DHT
  if (dhtSensor) {
    TempAndHumidity newValues;
    float heatIndex = -99;
    float dewPoint = -99;
    float cr = -99;

    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    // Check if any reads failed and exit early (to try again).
    if (dht.getStatus()) {
      DBG_ERROR("DHT11 error status : %s", dht.getStatusString());
    } else {
      newValues = dht.getTempAndHumidity();


      //record key value pairs on the measurementes array (to be published)
      temperature = newValues.temperature + Toffset;
      if (abs(temperature - last_temperature) > delta_t) {
        values["temperature"] = temperature;
        last_temperature = temperature;
      }

      humidity = adjRH(values["temperature"], newValues.humidity, Toffset);
      if (abs(humidity - last_humidity) > delta_h) {
        values["humidity"] = humidity;
        last_humidity = humidity;
      }
    }
  }
#endif
#ifdef BME280
  if (bmeSensor) {
    temperature = bme.readTemperature() + Toffset;
    if (abs(temperature - last_temperature) > delta_t) {
      values["temperature"] = temperature;
      last_temperature = temperature;
    }
    pressure = bme.readPressure();
    if (abs(pressure - last_pressure) > delta_p) {
      values["pressure"] = pressure;
      last_pressure = pressure;
    }
    humidity = adjRH(double(values["temperature"]), double(bme.readHumidity()), double(Toffset));
    if (abs(humidity - last_humidity) > delta_h) {
      values["humidity"] = humidity;
      last_humidity = humidity;
    }
  }
#endif
#ifdef WIFI_TELEMETRY
  wifi_db = getStrength(true);
  if (abs(wifi_db - last_wifi) > delta_w) {
    values["wifi_db"] = wifi_db;
    last_wifi = wifi_db;
  }
#endif
}

double adjRH(double tempAdjusted, double RH, double tempOffset) {
  double LRV, Temp, T0, E0, Es, Esadjusted, RHadjusted;
  LRV = 5423;   //K
  T0 = 273.15;  //K
  //E0 = 0.611; //kPa
  RH = RH / 100;
  tempAdjusted = tempAdjusted + T0;
  //Es = E0 * exp(LRV * (1 / T0 - 1 / Temp));
  Temp = tempAdjusted - tempOffset;
  //Esadjusted = E0 * exp(LRV * (1 / T0 - 1 / tempAdjusted));
  //RHadjusted = RH * Esadjusted / Es;
  RHadjusted = RH * exp(LRV * (1 / tempAdjusted - 1 / Temp));
  DBG_VERBOSE("RH %.1f%%, RH adj %.1f%%, Temp %.1f, Delta T %.1f",
              RH * 100, RHadjusted * 100, tempAdjusted, tempOffset);
  return RHadjusted * 100;
}

//  Publishes the measurements to the mqtt broker
bool publishMeasurement(JsonDocument &telemetry) {
  DBG_INFO("Start publishing");
  JsonObject values = telemetry["values"];
  for (JsonPair p : values) {
    DBG_INFO("key: %s value: %4.1f", p.key().c_str(), float(p.value()));
  }
  if (reconnect()) {
    char telemetry_str[200];
    serializeJson(values, telemetry_str);
    DBG_VERBOSE(telemetry_str);
    tb.sendTelemetryJson((const char *)telemetry_str);
    return true;
  } else return false;
}
//Function to display the measurements of the measuments array

void displayMeasurement(JsonDocument &telemetry) {
#ifdef DISPLAY
  if (displayPresent) {
    int lines = DISPLAY_LINES - 1;  // do not overflow display
    char line[20];
    display.clearDisplay();
    display.setTextSize(2);
    JsonObject values = telemetry["values"];
    lines /= 2;
    if (MAX_KEYS < lines) lines = MAX_KEYS;
    //display.clearDisplay();
    display.setCursor(0, 0);  //top left
    int j = 0;
    if (j < lines) {
      snprintf(line, sizeof(line), "CO2: %i", co2_ppm);
      display.println(line);
      j++;
    }
    if (j < lines) {
      snprintf(line, sizeof(line), "Hum: %-2.0f", float(humidity));
      display.println(line);
      j++;
    }
    if (j < lines) {
      snprintf(line, sizeof(line), "Tem: %-2.1f", temperature);
      display.println(line);
      j++;
    }
    if (connStatus.wifi) {
      display.drawBitmap(display.width() - 16, 16 - (connStatus.wifiStrength + 1) * 2,
                         wifi_icon16x2[16 - (connStatus.wifiStrength + 1) * 2], 16, (connStatus.wifiStrength + 1) * 2, 1);
      //display.setCursor(display.width() - 16, 16);
      //display.printf("%1i", connStatus.wifiStrength);
    } else display.drawBitmap(display.width() - 16, 0, cancel_icon16x16, 16, 16, 1);
    if (connStatus.mqtt) display.drawBitmap(display.width() - 16, 20, arrow_up_icon16x16, 16, 16, 1);
    else display.drawBitmap(display.width() - 16, 20, cancel_icon16x16, 16, 16, 1);
    display.setTextSize(1);
    display.setCursor(0, (DISPLAY_LINES - 1) * 8);  //last row
    if (userMessage != "") display.println(userMessage);
    else display.println(TOKEN);
    display.display();
  }
#endif
}
bool displayState = true;
//Activate leds
void showLeds(JsonDocument &telemetry) {
  //when enabled==1
  //led 0 lights if co2 value is above danger level (red)
  //led 1 lights if co2 is in between warning and danger levels (yellow)
  //led 2 lights if co2 is below warning level (green)
  //when enabled==0 led is off
  //when enabled==2 led is on

  DBG_VERBOSE("Levels warn: %i danger: %i Current CO2 level: %i ppm", level_warn, level_danger, co2_ppm);
  //turn off all leds
  for (int i = 0; i < 3; i++) digitalWrite(leds_control[i], 0);  //turn off all leds
  DBG_DEBUG("turn off all leds");
  if (displayState) {
    //turn on the level led
    if (co2_ppm <= level_warn) {
      DBG_DEBUG("Safe GREEN");
      digitalWrite(leds_control[2], 1);
    } else if ((co2_ppm <= level_danger)) {
      DBG_DEBUG("Warning Yellow");
      digitalWrite(leds_control[1], 1);
    } else {
      DBG_DEBUG("Danger RED");
      digitalWrite(leds_control[0], 1);
    }
    // If forced on or off
    for (int i = 0; i < 3; i++) {
      if (led_state[i] == 0) {
        DBG_DEBUG("Force off led: %i", i);
        digitalWrite(leds_control[i], 0);
      } else if (led_state[i] == 2) {
        DBG_DEBUG("Force on led: %i", i);
        digitalWrite(leds_control[i], 1);
      }
    }
  }
}
bool shouldSaveConfig = false;  //flag is changed by Callback function

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// obtain unique token for ESP32
void getToken() {
  // Compose TOKEN using chip id (mac address)
  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  snprintf(TOKEN, sizeof(TOKEN), "CaeliA-%04X%08X", chip, (uint32_t)chipid);
}

//The configuration parameters are stored in the config.json file
//in the the SPIF file system as a serialized json document.
//The function readd the configuration file into a json document
bool readConfigFile(JsonDocument &doc) {
  bool success = true;
  //DynamicJsonDocument doc(1024);
  DBG_INFO("mounting FS...");
  if (SPIFFS.begin()) {
    DBG_DEBUG("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      DBG_INFO("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        DBG_DEBUG("opened config file");
        deserializeJson(doc, configFile);
        serializeJson(doc, Serial);
        Serial.println();
        configFile.close();
      }
    }
  } else {
    DBG_ERROR("failed to mount FS...formatting and remount");
    SPIFFS.format();
    if (SPIFFS.begin()) DBG_INFO("mounting FS...");
    else DBG_ERROR("failed to mount FS... HW problem");
    success = false;
  }
  //end read
  return success;
  //return doc;
}

//write the configuration file from a json document
bool saveConfigFile(JsonDocument &doc) {
  bool success = true;
  DBG_INFO("saving config");
  //DynamicJsonDocument doc(1024);

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
    success = false;
  }
  serializeJson(doc, Serial);
  Serial.println();
  serializeJson(doc, configFile);
  configFile.close();
  //end save
  return success;
}

WiFiManager wm;

/*

*/

void InitWiFi() {
  char mqtt_server[40];
  char mqtt_port[8];
  char telemetry_topic[40];
  // char sw_server[40];
  // char sw_file[60];
  char temp_offset[8];
  DBG_INFO("Start wifi with autoconnect");
  getToken();
  //WiFiManager
  //set config save notify callback
  wm.setSaveConfigCallback(saveConfigCallback);

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  pinMode(PIN_CONFIGURE, INPUT);  //configure button to enable user to force the access point
#ifdef DISPLAY
  if (displayPresent) {
    display.setTextSize(2);
    display.println("Pulse para");
    display.println("configurar");
    display.display();
    delay(5000);  //wait 5 seconds to allow user to press the button
    display.clearDisplay();
    display.setCursor(0, 0);
    //display.println("Start");
    display.println("CaeliaAp");
    display.display();
    display.setTextSize(1);
  } else delay(5000);
#endif

  //clean FS, for testing
  //SPIFFS.format();
  //Serial.println("Formating FS...");

  //read configuration from FS json

  readConfigFile(configJson);
  wm.preloadWiFi((const char *)configJson["ap_ssid"], (const char *)configJson["ap_pass"]);
  if (!configJson["temp_offset"]) configJson["temp_offset"] = DEFAULT_TEMP_OFFSET;
  //saveConfigFile(configJson);

  wm.setConfigPortalTimeout(180);  //set 180s timeout for web portal
  WiFiManagerParameter mqttServer("Server_name", "mqtt_server", (const char *)configJson["mqtt_server"], 40);
  wm.addParameter(&mqttServer);
  //WiFiManagerParameter mqttPort("Port_number", "mqtt_port", (const char*)configJson["mqtt_port"], 6);
  //wm.addParameter(&mqttPort);
  //WiFiManagerParameter telemetryTopic("Telemetry", "Telemetry_topic", (const char*)configJson["telemetry_topic"], 40);
  //wm.addParameter(telemetryTopic);
  //WiFiManagerParameter swServer("Update_host", "Update_Server", (const char *)configJson["sw_server"], 40);
  //wm.addParameter(&swServer);
  //WiFiManagerParameter swFile("Update_file", "Update_file", (const char *)configJson["sw_file"], 60);
  //wm.addParameter(&swFile);
  WiFiManagerParameter tempOffset("Temp_offset", "Temp_offset", (const char *)configJson["temp_offset"], 6);
  wm.addParameter(&tempOffset);
  const char _customHtml_checkbox[] = "type = \"checkbox\"";
  WiFiManagerParameter autoCalibrate("checkbox", " Disable Auto Calibration", " ", 2, _customHtml_checkbox);
  wm.addParameter(&autoCalibrate);

  std::vector<const char *> menu = { "wifi", "erase", "restart", "exit" };
  wm.setMenu(menu);  // custom menu, pass vector

  wm.setWiFiAutoReconnect(true);  //autoreconnect

  DBG_INFO("Starting portal if needed");
  if (displayPresent) {
    display.println();
    display.println("Please connect to: ");
    display.println(TOKEN);
    display.display();
  }

  wm.setConfigPortalTimeout(180);  //set timeout to 180s
  wm.setConnectTimeout(60);
  bool connected;
  if (digitalRead(PIN_CONFIGURE)) {
    if (!(connected = wm.autoConnect(TOKEN))) {
      //wm.preloadWiFi("MiFibra-69A0", "cjFn5Q5J");
#ifdef RIVAS
      wm.preloadWiFi("Rivasciudad_InvitadosMAC", "");

#endif
    }
    DBG_INFO("Connected with autoconnect");
  } else {
    Serial.println("Forced configuration");
    SPIFFS.format();                          //clean FS
    connected = wm.startConfigPortal(TOKEN);  //Forced ap (user action)
    }

  //save the custom parameters to FS (if shouldSaveConfig has beenchanged through the callback function)
  if (shouldSaveConfig) {
    //DBG_INFO("Update file: %s", sw_file);
    configJson.clear();  //Clear all previous values
    strcpy(mqtt_server, mqttServer.getValue());

    //strcpy(sw_server, swServer.getValue());
    //strcpy(sw_file, swFile.getValue());
    strcpy(temp_offset, tempOffset.getValue());

    configJson["mqtt_server"] = mqtt_server;

    //configJson["sw_server"] = sw_server;
    //configJson["sw_file"] = sw_file;
    configJson["temp_offset"] = temp_offset;

    configJson["auto_calibration"] = (autoCalibrate.getValue()[0] == 'T') ? false : true;

    DBG_INFO("AutoCalibration: %s", autoCalibrate.getValue());
    //saveConfigFile(configJson);
  }
  if (connected) {
    //if you get here you have connected to the WiFi
    DBG_INFO("connected...yeey : )");
    //Save the credentials just in case we need to reconnect
    strcpy(WIFI_SSID, wm.getWiFiSSID().c_str());
    strcpy(WIFI_PASSWORD, wm.getWiFiPass().c_str());
    configJson["ap_ssid"] = WIFI_SSID;
    configJson["ap_pass"] = WIFI_PASSWORD;
    saveConfigFile(configJson);
  }
  DBG_INFO("AutoCalibration: %s", autoCalibrate.getValue());
#if ENCRYPTED
  espClient.setCACert(ROOT_CERT);
#endif      
  //WiFi.setSleep(true);  //enable sleep mode
}

//Check wifi connection. If WIFI connection is down try to reconnect
//and display icons on the screen to show the status of the wifi connection
//and the thingsboard connection.
bool reconnect()
// Check connection to Wifi and TB. If necessary reconnect
{
  bool success = true;  //succces retunt code
  //Check if we are connected
  if (WiFi.status() != WL_CONNECTED) {
    DBG_INFO("Disconnected from AP");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    connStatus.wifi = false;
    connStatus.mqtt = false;
    connStatus.rpc = false;
    success = false;
  } else {
    connStatus.wifi = true;
    connStatus.wifiStrength = getStrength(false);
    DBG_INFO("WiFI Signal Strength: %i", getStrength(true));
    // reset watchdog if we are connected
    if (wdtTimeout) timerWrite(timer, 0);  //reset timer (feed watchdog);
  }
  if (success) {
    // Reconnect to ThingsBoard, if needed
    if (!tb.connected()) {
      // reconnect to the ThingsBoard
      DBG_INFO("THINGSBOARD_PORT = %u", THINGSBOARD_PORT);
      DBG_INFO("Connecting to: %s with token: %s", (const char *)configJson["mqtt_server"], TOKEN);
#if ENCRYPTED
      espClient.setCACert(ROOT_CERT);
#endif  
      if (!tb.connect(configJson["mqtt_server"], TOKEN, THINGSBOARD_PORT)) {
      //if (!tb.connect(configJson["mqtt_server"], TOKEN)) {
        DBG_WARNING("Failed to connect to ThingsBoard ");
        connStatus.mqtt = false;
        connStatus.rpc = false;
      }
#ifdef RPC
      else {
        connStatus.mqtt = true;
        DBG_INFO("Subscribing for RPC...");
        // Perform a subscription. All consequent data processing will happen in
        // callbacks as denoted by callbacks[] array.
        if (!tb.RPC_Subscribe(callbacks.begin(), callbacks.end())) {
          DBG_WARNING("Failed to subscribe for RPC");
          success = false;
          connStatus.rpc = false;
        } else {
          DBG_INFO("Subscribe done");
          connStatus.rpc = true;
        }
      }
#endif
    } else connStatus.mqtt = true;
  }
  return success;
}

HardwareSerial mySerial(1);  // ESP32 serial port to communicate with sensor


/// @brief Updated callback that will be called as soon as the firmware update finishes
/// @param success Either true (update successful) or false (update failed)
void updatedCallback(const bool& success) {
  if (success) {
    DBG_INFO("Done, Reboot now");
    tb.Firmware_Send_State(FW_STATE_VERIFIED);
    delay(1000);
    esp_restart();
  }
  else {
    tb.Firmware_Send_State(FW_STATE_FAILED);
    DBG_INFO("Downloading firmware failed");
  }
  return;
}

/// @brief Progress callback that will be called every time we downloaded a new chunk successfully
/// @param currentChunk 
/// @param totalChuncks 

static void progressCallback(const size_t& currentChunk, const size_t& totalChuncks) {
  //Serial.printf("Progress %.2f%%\n", static_cast<float>(currentChunk * 100U) / totalChuncks);
  updating = true;
  Serial.printf("Progress %.2f%%\n", (currentChunk * 100.0 / totalChuncks));
}

#ifdef MHZ19_CO2
void setRange(int range) {
  myMHZ19.setRange(range);                                                // request new range write
  if ((myMHZ19.errorCode == RESULT_OK) && (myMHZ19.getRange() == range))  //RESULT_OK is an alias from the library,
  {
    DBG_INFO("Range : %i successfully applied.", range);  // Success
  } else {
    DBG_ERROR("Failed to set Range! Error Code : %i", myMHZ19.errorCode);  // Get the Error Code value
  }
}
#endif
void setup() {
 // Program the watchdog if wdtTimeout is not 0
  if (wdtTimeout) {
    timer = timerBegin(0, 8000, true);                //timer 0, demultiply by 8000 80Mhz clock = 0.1ms
    timerAttachInterrupt(timer, &resetModule, true);  //attach callback
    timerAlarmWrite(timer, wdtTimeout * 10, false);   //set time in units of 0.1 ms
    timerAlarmEnable(timer);                          //enable interrupt
  }
  //initialize configuration doc with preset values
  configJson["mqtt_server"] = THINGSBOARD_SERVER;
  configJson["mqtt_port"] = THINGSBOARD_PORT;
  configJson["telemetry_topic"] = TELEMETRY_TOPIC;
  //configJson["sw_server"] = SW_SERVER;
  //configJson["sw_file"] = SW_FILE;
  configJson["temp_offset"] = DEFAULT_TEMP_OFFSET;

  char myVersion[11];     // version for C02 sensor
  start_time = millis();  //power on time
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.flush();
  DBG_INFO("Booting");
  Wire.begin(I2C_SDA, I2C_SCL);
#ifdef DISPLAY
  Wire.beginTransmission(0x3C);
  if (!Wire.endTransmission()) displayPresent = true;
  if (displayPresent) {
    DBG_INFO("SSD1306 Display present");
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32

      DBG_ERROR("SSD1306 allocation failed");

      for (;;)
        ;  // Don't proceed, loop forever
    }
    display.setTextSize(3);               // Draw 3X-scale text
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.clearDisplay();
    display.setCursor(13, 0);  // Start at top-left corner
    display.println("CaeliA");
    display.display();
    display.setTextSize(1);  // Normal 1:1 pixel scale
  }
#endif

  InitWiFi();
  tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
  if (WiFi.status() == WL_CONNECTED) {
    //updateFW(true);
    timeClient.begin();
    timeClient.update();
    bootTimeStamp = timeClient.getEpochTime();
    DBG_INFO("Boot time: %s UTC", timeClient.getFormattedTime());
    Toffset = String((const char *)configJson["temp_offset"]).toFloat();
    DBG_INFO("Offset Temp: %.1f ", Toffset);
  }

  // initialize control pins for leds
  for (int i = 0; i < sizeof(leds_control); i++) pinMode(leds_control[i], OUTPUT);
  mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);  // device to CO2Sensor serial start
  mySerial.flush();

#ifdef CM1106_CO2
  //check for CM1106
  myCM1106.begin(mySerial);  // *Important, Pass your Stream reference
  myVersion[0] = 0;
  delay(5000);
  if (myCM1106.getCO2()) co2Sensor = CM1106sensor;
  //if (myVersion[0]) co2Sensor = CM1106sensor;
  //co2Sensor = CM1106sensor;
  if (co2Sensor == CM1106sensor) {
    DBG_INFO("CM1106 Sensor detected");
    delay(1000);
    myCM1106.autoCalibration(bool(configJson["auto_calibration"]), 7);
    delay(1000);
    myCM1106.getVersion(myVersion);
    DBG_VERBOSE("ProgrammID : ESP32_CM1106 Firmware Version : %s", myVersion);
  }
#endif
#ifdef MHZ19_CO2
  //check for MHZ19_CO2
  myMHZ19.begin(mySerial);  // *Important, Pass your Stream reference
  myVersion[0] = 0;
  myMHZ19.getVersion(myVersion);
  if (myVersion[0]) co2Sensor = MHZ19sensor;
  if (co2Sensor == MHZ19sensor) {
    DBG_INFO("MHZ19 Sensor detected");
    //delay(5000);                   // Wait for sensor to stabilize
    myMHZ19.autoCalibration(bool(configJson["auto_calibration"]));
    setRange(RANGE_Z19);  // set Range 5000 using a function, see below (disabled as part of calibration)
    //  Primary Information block
    DBG_VERBOSE("ProgrammID : ESP32_MHZ19 Firmware Version : %c%c.%c%c", myVersion[0], myVersion[1], myVersion[2], myVersion[3]);
    DBG_VERBOSE("Background CO2 : %i", myMHZ19.getBackgroundCO2());
    DBG_VERBOSE("Temperature Cal : %4.1f", myMHZ19.getTempAdjustment());
  }
#endif

#ifdef DHT
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT11);
  dht.getTempAndHumidity();
  if (dht.getStatus()) {
    DBG_DEBUG("DHT11 error status : %s", dht.getStatusString());
    DBG_INFO("DHT not detected");
    dhtSensor = 0;
  } else {
    dhtSensor = 1;
    DBG_INFO("DHT initiated");
  }
#endif
#ifdef BME280
  if (!bme.begin(0x76, &Wire)) {
    bmeSensor = 0;
    DBG_INFO("BME280 sensor not detected");
    //while (1); continue with no sensor
  } else {
    bmeSensor = 1;
    DBG_INFO("BME280 sensor initialized");
  }
#endif

#ifdef LP
  setCpuFrequencyMhz(80);  //Set CPU clock to 80MHz to save power
#endif
  DBG_INFO("CPU Frequency : %i MHz", getCpuFrequencyMhz());  //Get CPU clock
  //display.ssd1306_command(SSD1306_DISPLAYOFF);


  last_time = millis();
  display_time = millis();
  //Initiallize measurement tasks
  reconnect();
#ifdef DISPLAY
  if (displayPresent) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("Esperando medidas");
    display.setTextSize(1);
    display.display();
  }
#endif
  tb.Firmware_Send_State(FW_STATE_UPDATED); //Update firmware status
  //wait for warmup
  if (millis() - start_time < WARMUPTIME * 1000) {
    DBG_INFO("Waiting for : %i seconds for sensor warmup", (WARMUPTIME * 1000 - (millis() - start_time)) / 1000);
    delay((WARMUPTIME * 1000 - (millis() - start_time)));
  }
  DBG_INFO("-----------Starting measurement cycle-------------------");
}

bool pressed = false;
void loop() {
  delay(100);
  unsigned long current_time = millis();
  
  if (current_time - last_time > MEAS_PERIOD * 1000) {
    measTask();
    DBG_VERBOSE("Enable sleep : %s", (WiFi.getSleep()) ? "True" : "False");
    //WiFi.setSleep(true);
    last_time = current_time;
  }
#ifdef LP
  if (current_time - display_time > WAKE_PERIOD * 1000) {
    displayState = false;
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
#endif
  // Check if user presses the configure button in order to calibrate the device

  if (!digitalRead(PIN_CONFIGURE)) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    displayState = true;
    //trigger calibration if button is pressed for more than 5 secs
    if (pressed) {
      if (current_time - display_time > 5000) calibrationFlag = true;
    } else {
      DBG_INFO("Config PIN pressed");
      display_time = current_time;
    }
    pressed = true;
  } else pressed = false;
  
  if (!tb.connected()) {
    // Reconnect to the ThingsBoard server,
    // if a connection was disrupted or has not yet been established
    DBG_INFO("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      DBG_INFO("Failed to connect");
      return;
    }
  }
  if (!updating) {
    updateFW();
    //tb.Firmware_Send_State(FW_STATE_UPDATED);
  } 
  tb.loop();
}

unsigned long last_m = 0;
#define UPDATE_PERIOD 60 //seconds
void updateFW()
{
  unsigned long current_m;
  current_m = millis();
  if (last_m == 0) last_m = current_m-UPDATE_PERIOD*1000-1; //first execution
  if (((current_m - last_m) > UPDATE_PERIOD*1000))
  {
    updateRequestSent = false;
    if (forceupdateflag) {
      currentFWSent = false;
      DBG_INFO("Version: %s", FORCEUPDATE);
      /*
      if (!currentFWSent) {
        currentFWSent = tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, FORCEUPDATE) && tb.Firmware_Send_State(FW_STATE_UPDATED);
        DBG_INFO("Update info");
      }
      */
      if (!updateRequestSent) {
        DBG_INFO("Firwmare Update...");
        static const OTA_Update_Callback forcecallback(&progressCallback, &updatedCallback, CURRENT_FIRMWARE_TITLE,
                    FORCEUPDATE, &updater, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
      // See https://thingsboard.io/docs/user-guide/ota-updates/
        // to understand how to create a new OTA pacakge and assign it to a device so it can download it.
        updateRequestSent = tb.Start_Firmware_Update(forcecallback);
      }

    } else{
      DBG_INFO("Version: %s", CURRENT_FIRMWARE_VERSION);
      /*
      if (!currentFWSent) {
        // Firmware state send at the start of the firmware, to inform the cloud about the current firmware and that it was installed correctly,
        // especially important when using OTA update, because the OTA update sends the last firmware state as UPDATING, meaning the device is restarting
        // if the device restarted correctly and has the new given firmware title and version it should then send thoose to the cloud with the state UPDATED,
        // to inform any end user that the device has successfully restarted and does actually contain the version it was flashed too
        tb.Firmware_Send_State(FW_STATE_UPDATED);
        currentFWSent = tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
        DBG_INFO("Update info");
      }
      */
      if (!updateRequestSent) {
        DBG_INFO("Firwmare Update...");
        static const OTA_Update_Callback fwcallback(&progressCallback, &updatedCallback, CURRENT_FIRMWARE_TITLE,
                    CURRENT_FIRMWARE_VERSION, &updater, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
        // See https://thingsboard.io/docs/user-guide/ota-updates/
        // to understand how to create a new OTA pacakge and assign it to a device so it can download it.
        updateRequestSent = tb.Start_Firmware_Update(fwcallback);
        // delay(1000);
        // tb.Firmware_Send_State(FW_STATE_UPDATED);
      }

    }
    last_m = current_m;
  }
}

