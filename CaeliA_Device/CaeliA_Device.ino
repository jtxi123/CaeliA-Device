#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
//#define DBG_ENABLE_DEBUG
#define DBG_ENABLE_VERBOSE
//Debug level for WiFiManager

#include <ArduinoDebug.hpp>
DEBUG_INSTANCE(200, Serial);

#include <FS.h>
#include <SPIFFS.h>
#include <esp32fota.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ThingsBoard.h>    // ThingsBoard SDK

#include <NTPClient.h>
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
//
//tiempo UTC para tener hora local hy que modificar el
// offset (0) con la función timeClient.setTimeOffset(long timeOffset)
//NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 60000);
NTPClient timeClient(ntpUDP);

//needed for library for autoconnect
#include <DNSServer.h>
#include <WebServer.h>
#define WM_DEBUG_LEVEL 0
#include <WiFiManager.h>

//SW type and version needed to check for SW updates
#define SW_TYPE "CaeliA_esp32_co2"
#define SW_VERSION 1
#define SW_SERVER "http://192.168.1.200:80/fota/fota.json"
esp32FOTA esp32FOTA(SW_TYPE, SW_VERSION);
#define WARN 700;
#define DANGER 1500;

//#include <credentials.h>
char ap_ssid[40]; //to store the wifi ap name
char ap_pass[40]; //to store the wifi ap password
#define MHZ19_CO2
#define CM1106_CO2
#define RPC
//device has SSD1306 128x64 display
#define DISPLAY

#define BAUDRATE 9600           // Device to CO2 sensor Serial baudrate (should not be changed)
#define RX_PIN 19               // Rx pin which the CO2 Sensor Tx pin is attached
#define TX_PIN 18               // Tx pin which the CO2 Sensor Rx pin is attached 

#ifdef MHZ19_CO2
#include "MHZ19.h"
#define RANGE_Z19 5000          // Range for CO2 readings 
MHZ19 myMHZ19;                  // Constructor for library
#endif

#ifdef CM1106_CO2
#include "CM1106.h"               //Using CM1106 Sensor
CM1106 myCM1106;                  //Constructor for library
#endif
typedef enum SensorType
{
  NOSENSOR = 0,     //No CO2 sensor
  MHZ19sensor = 1,      // 0 MHZ19 sensor
  CM1106sensor = 2        // 1 CM1106 sensor
} SensorType;
SensorType co2Sensor = NOSENSOR;

#define DHT                     //DHT temperature and hunidity sensor
#ifdef DHT
#include "DHTesp.h"
DHTesp dht;
// Comfort profile
ComfortState cf;
// Pin number for DHT11 data pin
int dhtPin = 23;
int dhtSensor = 0;
#endif

#define BME280
#ifdef BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
int bmeSensor = 0;
#endif


#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

#ifdef DISPLAY
//#include <SPI.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//#define I2C_SDA 21
//#define I2C_SCL 22
#define I2C_SDA 5
#define I2C_SCL 4
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

//Create the display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//Bitmap Icons for display
#define wifi1_icon16x16_width 16
#define wifi1_icon16x16_height 16
//variable to register the detection of a display at address 0x3C
bool displayPresent = false;
unsigned char wifi1_icon16x16[] =
{
  0b00000000, 0b00000000, //
  0b00000111, 0b11100000, //      ######
  0b00011111, 0b11111000, //    ##########
  0b00111111, 0b11111100, //   ############
  0b01110000, 0b00001110, //  ###        ###
  0b01100111, 0b11100110, //  ##  ######  ##
  0b00001111, 0b11110000, //     ########
  0b00011000, 0b00011000, //    ##      ##
  0b00000011, 0b11000000, //       ####
  0b00000111, 0b11100000, //      ######
  0b00000100, 0b00100000, //      #    #
  0b00000001, 0b10000000, //        ##
  0b00000001, 0b10000000, //        ##
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
};

#define arrow_up_icon16x16_width 16
#define arrow_up_icon16x16_height 16
unsigned char arrow_up_icon16x16[] =
{
  0b00000001, 0b10000000, //        ##
  0b00000011, 0b11000000, //       ####
  0b00000111, 0b11100000, //      ######
  0b00001111, 0b11110000, //     ########
  0b00011110, 0b01111000, //    ####  ####
  0b00111100, 0b00111100, //   ####    ####
  0b01111000, 0b00011110, //  ####      ####
  0b11111100, 0b00111111, // ######    ######
  0b11111100, 0b00111111, // ######    ######
  0b01111100, 0b00111110, //  #####    #####
  0b00011100, 0b00111000, //    ###    ###
  0b00011100, 0b00111000, //    ###    ###
  0b00011100, 0b00111000, //    ###    ###
  0b00011111, 0b11111000, //    ##########
  0b00011111, 0b11111000, //    ##########
  0b00001111, 0b11110000, //     ########
};

#define cancel_icon16x16_width 16
#define cancel_icon16x16_height 16
unsigned char cancel_icon16x16[] =
{
  0b00000000, 0b00000000, //
  0b00000000, 0b00000000, //
  0b00111000, 0b00001110, //   ###       ###
  0b00111100, 0b00011110, //   ####     ####
  0b00111110, 0b00111110, //   #####   #####
  0b00011111, 0b01111100, //    ##### #####
  0b00001111, 0b11111000, //     #########
  0b00000111, 0b11110000, //      #######
  0b00000011, 0b11100000, //       #####
  0b00000111, 0b11110000, //      #######
  0b00001111, 0b11111000, //     #########
  0b00011111, 0b01111100, //    ##### #####
  0b00111110, 0b00111110, //   #####   #####
  0b00111100, 0b00011110, //   ####     ####
  0b00111000, 0b00001110, //   ###       ###
  0b00000000, 0b00000000, //
};
#endif



// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

char token[40];
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "demo.thingsboard.io"
//#define THINGSBOARD_SERVER  "192.168.1.100"
#define TELEMETRY_TOPIC "v1/devices/me/telemetry"
#define THINGSBOARD_PORT "1883"

// Baud rate for debug serial console
#define SERIAL_DEBUG_BAUD    115200

#define MAX_KEYS 20 //Maximum number of Telemetry key-value pairs
#define NO_KEY ""  //empty key tag name 
#define DISPLAY_LINES 8 //Number of lines on display


// Create the ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int wifiStatus = WL_IDLE_STATUS;

DynamicJsonDocument configJson(1024); //json document to store config parameters
//StaticJsonDocument<200> configJson; //json document to store config parameters
#define RED 15
#define YELLOW 2
#define GREEN 0
// Array with LEDs that should be controlled from ThingsBoard
uint8_t leds_control[] = { RED, YELLOW, GREEN };

#define PIN_CONFIGURE 3 //Pin used to force configuration at boot
#define PIN_BOOT 0

//Telemetry Json document
DynamicJsonDocument telemetry(1024);
//Telemetry telemetry[MAX_KEYS];
int keyIndex;

#ifdef RPC
// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;

// Processes functions for RPC calls
// RPC_Data is a JSON variant, that can be queried using operator[]
// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
// Function to modify offset to adjust co2 measurments.
int co2Offset = 0;
RPC_Response processSetOffset(const RPC_Data &data)
{
  // Process data
  co2Offset += int(data); //Change co2 offset

  DBG_INFO("Received the set delay RPC method");
  DBG_INFO("Set new CO2_offset: %i", co2Offset);

  return RPC_Response(NULL, co2Offset);
}

// Function to turn on or off auto calibration.
RPC_Response processAutoCalibration(const RPC_Data &data)
{
  if (data) {
    DBG_INFO("Set auto calibration ON");
#ifdef MHZ19_CO2
    if (co2Sensor == MHZ19sensor) {
      //Check if a calibration is needed
      myMHZ19.autoCalibration(true);
      // set Range to RANGE_Z19 using a function, see below (disabled as part of calibration)
      myMHZ19.setRange(RANGE_Z19);
    }
#endif
#ifdef CM1106_CO2
    if (co2Sensor == CM1106sensor) {
      myCM1106.autoCalibration(true, 7);
    }
#endif
  } else {
    DBG_INFO("Set auto calibration OFF");
#ifdef MHZ19_CO2
    if (co2Sensor == MHZ19sensor) {
      myMHZ19.autoCalibration(true);
      // set Range to RANGE_Z19 using a function, see below (disabled as part of calibration)
      myMHZ19.setRange(RANGE_Z19);
    }
#endif
#ifdef CM1106_CO2
    if (co2Sensor == CM1106sensor) {
      //Check if a calibration is needed
      myCM1106.autoCalibration(true, 7);
    }
#endif
  }
  return RPC_Response("AutoCalibration", bool(data));
}

bool calibrationFlag;
RPC_Response processCalibration(const RPC_Data &data)
{
  calibrationFlag = true;
  DBG_INFO("Received the autoCalibration RPC command");
  DBG_INFO("Set the calibration flag: %i", calibrationFlag);
  return RPC_Response("calibration", calibrationFlag);
}

// Function to receive a message with a severity level.
String userMessage = "";
int messageLevel = 0;
RPC_Response processUserMessage(const RPC_Data &data)
{
  const char* message;

  DBG_INFO("Received the get userMessage method");

  message = data["message"];
  userMessage = String(message);
  messageLevel = data["level"];

  DBG_INFO("Mensaje: %s Nivel: %i", message, messageLevel);

  return RPC_Response("message", message);
}

// Function to control leds
// when led_state ==1 led will light in accordance with the CO2 level
// when led_state ==2 led is always on
// when led_state ==0 the led is always off

int led_state[3] = {1, 1, 1};

RPC_Response processSetLedState(const RPC_Data &data)
{
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
  return RPC_Response("enabled", enabled);
}

int level_warn = WARN;
int level_danger = DANGER;

RPC_Response processSetWarnings(const RPC_Data & data)
{
  Serial.println("Received the set setWarnings RPC method");

  level_warn = data["warning"];
  level_danger = data["danger"];

  DBG_INFO("Setting Warning levels");
  DBG_INFO("From 0 to %i ppm good", level_warn);
  DBG_INFO("From %i to %i ppm fair", level_warn + 1, level_danger);
  DBG_INFO("Above %i  ppm danger", level_danger);

  return RPC_Response("warning", level_warn);
}

// RPC handlers for RPC
RPC_Callback callbacks[] = {
  {"setOffset",         processSetOffset },
  {"calibration",       processCalibration },
  {"autoCalibration",    processAutoCalibration },
  {"userMessage",       processUserMessage },
  {"setLedState",       processSetLedState },
  {"setWarnings",       processSetWarnings },
};
#endif

// This measurments period MEAS_PERIOD in seconds

#define MEAS_PERIOD 20


unsigned long start_time; //record the initial millis() to wait for sensor stabilization
#define WARMUPTIME 60


//  Task to reads & precess measurements from sensors



void measTask() {

  DBG_INFO("measTask started");
  display.clearDisplay();
  readMeasurement();
  displayMeasurement();
  publishMeasurement();
  showLeds();
  calibration();
  DBG_INFO("measTask ended");

}

//Activate leds
void showLeds()
{
  //when enabled==1
  //led 0 lights if co2 value is above danger level (red)
  //led 1 lights if co2 is in between warning and danger levels (yellow)
  //led 2 lights if co2 is below warning level (green)
  //when enabled==0 led is off
  //when enabled==2 led is on

  int co2_ppm = telemetry["values"]["co2_ppm"];
  DBG_VERBOSE("Levels warn: %i danger: %i Current CO2 level: %i ppm", level_warn, level_danger, co2_ppm);
  //turn off all leds
  for (int i = 0; i < 3; i++) digitalWrite(leds_control[i], 0);//turn off all leds
  DBG_DEBUG("turn off all leds");
  //turn on the level led
  if (co2_ppm <= level_warn) {
    DBG_DEBUG("Safe GREEN");
    digitalWrite(leds_control[2], 1);
  }
  else if ((co2_ppm <= level_danger) ) {
    DBG_DEBUG("Warning Yellow");
    digitalWrite(leds_control[1], 1);
  }
  else {
    DBG_DEBUG("Danger RED");
    digitalWrite(leds_control[0], 1);
  }
  // If forced on or off
  for (int i = 0; i < 3; i++)
  {
    if (led_state[i] == 0) {
      DBG_DEBUG("Force off led: %i", i);
      digitalWrite(leds_control[i], 0);
    }
    else if (led_state[i] == 2) {
      DBG_DEBUG("Force on led: %i", i);
      digitalWrite(leds_control[i], 1);
    }
  }
}

void calibration() //Calibrate if needed
{
  if (calibrationFlag) {
#ifdef MHZ19_CO2
    if (co2Sensor == MHZ19sensor) {
      //Check if a calibration is needed
      myMHZ19.calibrate();
      myMHZ19.autoCalibration(true);
      // set Range to RANGE_Z19 using a function, see below (disabled as part of calibration)
      myMHZ19.setRange(RANGE_Z19);
    }
#endif
#ifdef CM1106_CO2
    if (co2Sensor == CM1106sensor) {
      //Check if a calibration is needed
      myCM1106.calibrateZero();
      myCM1106.autoCalibration(true, 7);
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
void readMeasurement()
{
  int co2_ppm; //concertration of CO2
  float co2_t; //Temperature of the sensor (not very accurate as sensor generates heat)
  keyIndex = 0;
  timeClient.update();
  unsigned long long ts;
  char ts_str[20];
  JsonObject tm = telemetry.to<JsonObject>();
  ts = timeClient.getEpochTime();
  ts = ts * 1000; // in ms? UTC
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

    co2_ppm = co2Offset + myMHZ19.getCO2(); // Request CO2 (as ppm) unlimimted value, new request
    //co2_ppm = co2Offset + myMHZ19.getCO2(true, true); // Request CO2 (as ppm) unlimimted value, new request
    co2_t = myMHZ19.getTemperature(true, false);   // decimal value, not new request
    //co2_t = myMHZ19.getTempAdjustment();   // adjusted temperature

    values["co2_ppm"] = co2_ppm;
    keyIndex++;

    values["co2_t"] = co2_t;
    keyIndex++;

    values["co2_raw"] = myMHZ19.getCO2Raw();
    keyIndex++;

    double adjustedCO2 = myMHZ19.getCO2Raw();
    adjustedCO2 = 6.60435861e+15 * exp(-8.78661228e-04 * adjustedCO2);      // Exponential equation for Raw & CO2 relationship
    values["co2_adj"] = adjustedCO2;
    keyIndex++;
  }
#endif
#ifdef CM1106_CO2
  if (co2Sensor == CM1106sensor) {
    co2_ppm = co2Offset + myCM1106.getCO2(); // Request CO2 (as ppm) unlimimted value, new request
    if (!myCM1106.getStatus()) DBG_INFO("CO2 sensor preheating");
    else {
      values["co2_ppm"] = co2_ppm;
      keyIndex++;
    }
  }
#endif

#ifdef DHT
  if (dhtSensor)
  {
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
      heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
      dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
      cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

      //record key value pairs on the measurementes array (to be published)
      values["temperature"] = newValues.temperature;
      keyIndex++;

      values["humidity"] = newValues.humidity;
      keyIndex++;

      values["dewPoint"] = dewPoint;
      keyIndex++;

      values["dewPoint"] = dewPoint;
      keyIndex++;

      values["comfortRatio"] = cr;
      keyIndex++;
    }
  }
#endif
#ifdef BME280
  if (bmeSensor)
  {
    values["temperature"] = bme.readTemperature();
    keyIndex++;

    values["pressure"] = bme.readPressure();
    keyIndex++;

    values["humidity"] = bme.readHumidity();
    keyIndex++;

    values["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    keyIndex++;
  }
#endif

}

//  Publishes the measurements to the mqtt broker
bool publishMeasurement()
{

  DBG_INFO("Start publishing");
  JsonObject values = telemetry["values"];
  for (JsonPair p : values) {
    DBG_INFO("key: %s value: %4.1f", p.key().c_str(), float(p.value()));
  }
  if (checkConnection()) {
    char telemetry_str[200];
    serializeJson(telemetry, telemetry_str);
    DBG_INFO(telemetry_str);
    tb.sendTelemetryJson((const char*)telemetry_str);
    return true;
  }
  else return false;
}
//Function to display the measurements of the measuments array

void displayMeasurement()
{

#ifdef DISPLAY
  if (displayPresent)
  {
    int lines = DISPLAY_LINES - 1; // do not overflow display
    char line[20];
    display.clearDisplay();
    display.setTextSize(2);
    JsonObject values = telemetry["values"];
    lines /= 2;
    if (MAX_KEYS < lines) lines = MAX_KEYS;
    //display.clearDisplay();
    display.setCursor(0, 0); //top left
    int j = 0;
    if ( j < lines) {
      snprintf(line, sizeof(line), "CO2: %i", int(values["co2_ppm"]));
      display.println(line);
      j++;
    }
    if ( j < lines) {
      snprintf(line, sizeof(line), "Hum: %-2.0f", float(values["humidity"]));
      display.println(line);
      j++;
    }
    if ( j < lines) {
      snprintf(line, sizeof(line), "Tem: %-2.1f", float(values["temperature"]));
      display.println(line);
      j++;
    }
    display.setTextSize(1);
    display.setCursor(0, (DISPLAY_LINES - 1) * 8); //last row
    if (userMessage != "") display.println(userMessage);
    else display.println(token);
    display.display();
  }
#endif

}


// Check for software updates
// The function execHTTPcheck reads the json document in the sw_server
// and checks if therere is a higher version.
// in such case it executes the execOTA function that downloads and installs
// the new version.
unsigned long last_m = 0;
#define UPDATE_PERIOD 60 //seconds
void updateSW()
{

  unsigned long current_m;
  current_m = millis();
  if (last_m == 0) last_m = current_m; //first execution
  if ((current_m - last_m) / 1000 > UPDATE_PERIOD)
  {
    DBG_INFO("Check for updates Current Type : %s  Version : %i", SW_TYPE, SW_VERSION);
    if (esp32FOTA.execHTTPcheck()) esp32FOTA.execOTA();
    last_m = current_m;
  }

}


bool shouldSaveConfig = false; //flag is changed by Callback function

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// obtain unique token for ESP32
void getToken()
{
  // Compose TOKEN using chip id (mac address)
  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  snprintf(token, sizeof(token), "CaeliA-%04X%08X", chip, (uint32_t)chipid);
}

//The configuration parameters are stored in the config.json file
//in the the SPIF file system as a serialized json document.
//The function readd the configuration file into a json document
bool readConfigFile(JsonDocument& doc)
{
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
    DBG_ERROR("failed to mount FS");
    success = false;
  }
  //end read
  return success;
  //return doc;
}

//write the configuration file from a json document
bool saveConfigFile(JsonDocument& doc)
{
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

WiFiManager wifiManager;
void InitWiFi()
{
  char mqtt_server[40];
  char mqtt_port[8];
  char telemetry_topic[40];
  char sw_server[40];
  getToken();
  //WiFiManager

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //reset saved settings
  //wifiManager.resetSettings();
  //set custom ip for portal
  //wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  pinMode(PIN_CONFIGURE, INPUT); //configure button to enable user to force the access point
#ifdef DISPLAY
  if (displayPresent)
  {
    display.setTextSize(2);
    display.println("Pulse para");
    display.println("configurar");
    display.display();
    delay(5000); //wait 5 seconds to allow user to press the button
    display.clearDisplay();
    display.setCursor(0, 0);
    //display.println("Start");
    display.println("CaeliaAp");
    display.display();
    display.setTextSize(1);
  }
#endif

  //clean FS, for testing
  //SPIFFS.format();
  //Serial.println("Formating FS...");

  //read configuration from FS json

  readConfigFile(configJson);

  wifiManager.setConfigPortalTimeout(180); //set 180s timeout for web portal
  WiFiManagerParameter mqttServer("Server_name", "mqtt_server", (const char*)configJson["mqtt_server"], 40);
  wifiManager.addParameter(&mqttServer);
  WiFiManagerParameter mqttPort("Port_number", "mqtt_port", (const char*)configJson["mqtt_port"], 6);
  wifiManager.addParameter(&mqttPort);
  WiFiManagerParameter telemetryTopic("Telemetry", "Telemetry_topic", (const char*)configJson["telemetry_topic"], 40);
  wifiManager.addParameter(&telemetryTopic);
  WiFiManagerParameter swServer("Update", "Update_Server", (const char*)configJson["sw_server"], 40);
  wifiManager.addParameter(&swServer);

  DBG_INFO("Starting portal if needed");
  if (displayPresent)
  {
    display.println();
    display.println("Please connect to: ");
    display.println(token);
    display.display();
  }
  wifiManager.setConfigPortalTimeout(180);//set timeout to 180s
  wifiManager.setConnectTimeout(60);
  if (digitalRead(PIN_CONFIGURE))
  {
    wifiManager.autoConnect(token);
  }
  else
  {
    Serial.println("Forced configuration");
    SPIFFS.format(); //clean FS
    wifiManager.startConfigPortal(token); //Forced ap (user action)
  }
  //or use this for auto generated name ESP + ChipID

  //save the custom parameters to FS
  if (shouldSaveConfig) {

    strcpy(mqtt_server, mqttServer.getValue());
    strcpy(mqtt_port, mqttPort.getValue());
    strcpy(telemetry_topic, telemetryTopic.getValue());
    strcpy(sw_server, swServer.getValue());

    configJson["mqtt_server"] = mqtt_server;
    configJson["mqtt_port"] = mqtt_port;
    configJson["telemetry_topic"] = telemetry_topic;
    configJson["sw_server"] = sw_server;
    saveConfigFile(configJson);
  }


#ifdef DISPLAY
  if (displayPresent)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("Esperando medidas");
    display.setTextSize(1);
    display.display();
  }
#endif
  //if you get here you have connected to the WiFi
  DBG_INFO("connected...yeey : )");
  //Save the credentials just in case we need to reconnect
  strcpy(ap_ssid, wifiManager.getWiFiSSID().c_str());
  strcpy(ap_pass, wifiManager.getWiFiPass().c_str());
  //WiFi.disconnect(); //disconnect and reconnect to make sure we are on STA mode
  //WiFi.mode(WIFI_STA); // disable AP for modemsleep
  //checkConnection();
}

//Check wifi connection. If WIFI connection is down try to reconnect
//and display icons on the screen to show the status of the wifi connection
//and the thingsboard connection.
bool checkConnection()
// Check connection to Wifi and TB. If necessary reconnect
{

  bool success = true; //succces retunt code
  //Check if we are connected
  if (WiFi.status() != WL_CONNECTED)
  {
    DBG_INFO("Disconnected from AP");
    if (displayPresent)
    {
      display.drawBitmap(display.width() - 16, 0, cancel_icon16x16, 16, 16, 1);
      display.display();
    }
    WiFi.begin(ap_ssid, ap_pass);
    success = false;
  } else {
    if (displayPresent)
    {
      display.drawBitmap(display.width() - 16, 0, wifi1_icon16x16, 16, 16, 1);
      display.display();
    }

    // Reconnect to ThingsBoard, if needed
    if (!tb.connected()) {
      if (displayPresent)
      {
        display.drawBitmap(display.width() - 16, 16, cancel_icon16x16, 16, 16, 1);
        display.display();
      }
      success = false;
      // Connect to the ThingsBoard
      DBG_INFO("Connecting to: %s with token: %s", (const char*)configJson["mqtt_server"], token);

      if (!tb.connect(configJson["mqtt_server"], token)) {
        DBG_WARNING("Failed to connect to ThingsBoard ");
        success = false;
      }
#ifdef RPC
      DBG_INFO("Subscribing for RPC...");
      // Perform a subscription. All consequent data processing will happen in
      // callbacks as denoted by callbacks[] array.
      if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
        DBG_WARNING("Failed to subscribe for RPC");
        success = false;
      }
      DBG_INFO("Subscribe done");
#endif
    }
#ifdef DISPLAY
    if (displayPresent)
    {
      if (success) display.drawBitmap(display.width() - 16, 16, arrow_up_icon16x16, 16, 16, 1);
      else display.drawBitmap(display.width() - 16, 16, cancel_icon16x16, 16, 16, 1);
      display.display();
    }
#endif
  }
  return success;
}

HardwareSerial mySerial(1);     // ESP32 serial port to communicate with sensor
bool tasksEnabled = false;
unsigned long current_time;
unsigned long last_time;

// Setup procedure
void setup()
{

  //initialize configuration doc with preset values
  configJson["mqtt_server"] = THINGSBOARD_SERVER;
  configJson["mqtt_port"] = THINGSBOARD_PORT;
  configJson["telemetry_topic"] = TELEMETRY_TOPIC;
  configJson["sw_server"] = SW_SERVER;
  char myVersion[11];              // version for C02 sensor
  start_time = millis(); //power on time
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.flush();
  DBG_INFO("Booting");
  Wire.begin(I2C_SDA, I2C_SCL);
#ifdef DISPLAY
  Wire.beginTransmission(0x3C);
  if (!Wire.endTransmission()) displayPresent = true;
  if (displayPresent)
  {
    DBG_INFO("SSD1306 Display present");
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32

      DBG_ERROR("SSD1306 allocation failed");

      for (;;); // Don't proceed, loop forever
    }
    display.setTextSize(3);             // Draw 3X-scale text
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.clearDisplay();
    display.setCursor(13, 0);            // Start at top-left corner
    display.println("CaeliA");
    display.display();
    display.setTextSize(1);             // Normal 1:1 pixel scale
  }
#endif
  InitWiFi();
  timeClient.begin();
  timeClient.update();
  DBG_INFO("Boot time: %s UTC", timeClient.getFormattedTime());

  //ennable uploading over the air
  //register the server where to check if firmware needs updating
  esp32FOTA.checkURL = String((const char*)configJson["sw_server"]);
  // initialize control pins for leds
  for (int i = 0; i < sizeof(leds_control); i++) pinMode(leds_control[i], OUTPUT);
  mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);   // device to CO2Sensor serial start
  mySerial.flush();

#ifdef CM1106_CO2
  //check for CM1106
  myCM1106.begin(mySerial);       // *Important, Pass your Stream reference
  myVersion[0] = 0;
  myCM1106.getVersion(myVersion);
  if (myVersion[0]) co2Sensor = CM1106sensor;
  if (co2Sensor == CM1106sensor) {
    DBG_INFO("CM1106 Sensor detected");
    DBG_VERBOSE("ProgrammID : ESP32_CM1106 Firmware Version : %s", myVersion);
    myCM1106.autoCalibration(true, 7);
  }
#endif
#ifdef MHZ19_CO2
  //check for MHZ19_CO2
  myMHZ19.begin(mySerial);       // *Important, Pass your Stream reference
  myVersion[0] = 0;
  myMHZ19.getVersion(myVersion);
  if (myVersion[0]) co2Sensor = MHZ19sensor;
  if (co2Sensor == MHZ19sensor) {
    DBG_INFO("MHZ19 Sensor detected");
    //delay(5000);                   // Wait for sensor to stabilize
    myMHZ19.autoCalibration(true);
    setRange(RANGE_Z19);                // set Range 5000 using a function, see below (disabled as part of calibration)
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

  delay(2000); // Pause for 2 seconds

  //Check if update is needed
  DBG_DEBUG('(const char*)(configJson["sw_server"])[0] = %c', (const char*)(configJson["sw_server"])[0]);
  if (((const char*)(configJson["sw_server"]))[0]) if (esp32FOTA.execHTTPcheck()) esp32FOTA.execOTA();

  //Initiallize measurement tasks
  checkConnection();

  // Signal end of setup() to tasks
  tasksEnabled = false;
  //wait for warmup
  if (millis() - start_time < WARMUPTIME * 1000)
  {
    DBG_INFO("Waiting for : %i seconds for sensor warmup", (WARMUPTIME * 1000 - (millis() - start_time)) / 1000);
    delay((WARMUPTIME * 1000 - (millis() - start_time)));
  }
  last_time = millis();
  DBG_INFO("-----------Starting measurement cycle-------------------");
}

void loop() {
  int current_time = millis();
  if (current_time - last_time > MEAS_PERIOD * 1000)
  {
    measTask();
    last_time = current_time;
  }

  // Check if user presses the configure button in order to calibrate the device
  if (!digitalRead(PIN_CONFIGURE)) calibrationFlag = true;
  if (((const char*)(configJson["sw_server"]))[0])updateSW(); //check for sw updates
#ifdef RPC
  //check for incomming messages
  tb.loop();
#endif
  delay(1000);
  //yield();
}
#ifdef MHZ19_CO2
void setRange(int range)
{
  myMHZ19.setRange(range);                        // request new range write
  if ((myMHZ19.errorCode == RESULT_OK) && (myMHZ19.getRange() == range))     //RESULT_OK is an alias from the library,
  {
    DBG_INFO("Range : %i successfully applied.", range); // Success
  }
  else
  {
    DBG_ERROR("Failed to set Range! Error Code : %i", myMHZ19.errorCode);         // Get the Error Code value
  }
}
#endif
