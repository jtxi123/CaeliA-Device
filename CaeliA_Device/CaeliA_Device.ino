#include <FS.h>
#include <SPIFFS.h>
#include <esp32fota.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ThingsBoard.h>    // ThingsBoard SDK

//needed for library for autoconnect
#include <DNSServer.h>
#include <WebServer.h>
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
//#define CM1106_CO2
#define RPC
//device has SSD1306 128x64 display
#define DISPLAY

#define BAUDRATE 9600           // Device to CO2 sensor Serial baudrate (should not be changed)
#define RX_PIN 19               // Rx pin which the CO2 Sensor Tx pin is attached
#define TX_PIN 18               // Tx pin which the CO2 Sensor Rx pin is attached

#ifdef MHZ19_CO2
#include "MHZ19.h"
#define RANGE_Z19 2000          // Range for CO2 readings 
MHZ19 myMHZ19;                  // Constructor for library
char myVersion[4];              // needed for MH-Z19B
#endif

#ifdef CM1106_CO2
#include "CM1106.h"               //Using CM1106 Sensor
CM1106 myCM1106;                  //Constructor for library
#endif

#define DHT                     //DHT temperature and hunidity sensor
#ifdef DHT
#include "DHTesp.h"
#endif

//#define BME280
#ifdef BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
#endif


#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

#ifdef DISPLAY
#include <SPI.h>
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

// See https://thingsboard.io/docs/getting-started-guides/helloworld/

// to understand how to obtain an access token
//#define TOKEN  "ESP32_DEMO_TOKEN_JG" //to be changed
char token[40];
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "demo.thingsboard.io"
//#define THINGSBOARD_SERVER  "192.168.1.100"
#define TELEMETRY_TOPIC "v1/devices/me/telemetry"
#define THINGSBOARD_PORT "1883"

// Baud rate for debug serial console
#define SERIAL_DEBUG_BAUD    115200
#define DEBUG

#define MAX_KEYS 10 //Maximum number of Telemetry key-value pairs
#define NO_KEY ""  //empty key tag name 
#define DISPLAY_LINES 8 //Number of lines on display

//Measurement structure (key value pairs)
struct MEASUREMENT
{
  String  key;
  float value;
};

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
#ifdef DEBUG
  Serial.println("Received the set delay RPC method");
  Serial.print("Set new CO2_offset: ");
  Serial.println(co2Offset);
#endif
  return RPC_Response(NULL, co2Offset);
}

// Function to schedule a new calibration.
bool calibrationFlag;
RPC_Response processCalibration(const RPC_Data &data)
{
  calibrationFlag = true;
#ifdef DEBUG
  Serial.println("Received the udr message command");
  Serial.print("Set the calibration flag: ");
  Serial.println(calibrationFlag);
#endif
  return RPC_Response("calibration", calibrationFlag);
}

// Function to receive a message with a severity level.
String userMessage = "";
int messageLevel = 0;
RPC_Response processUserMessage(const RPC_Data &data)
{
  const char* message;
#ifdef DEBUG
  Serial.println("Received the get userMessage method");
#endif
  message = data["message"];
  userMessage = String(message);
  messageLevel = data["level"];
#ifdef DEBUG
  Serial.println("Mensaje: " + userMessage + " Nivel: " + String(messageLevel));
#endif
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
  serializeJson(data,Serial);
  int led = data["led"];
  int enabled = data["enabled"];

  if (led < COUNT_OF(leds_control)) {
#ifdef DEBUG
    Serial.print("Setting LED ");
    Serial.print(String(led) + " (GPIO" + String(leds_control[led]) + ") ");
    Serial.print(" to state ");
    Serial.println(enabled);
#endif
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

#ifdef DEBUG
  Serial.print("Setting Warning levels");
  Serial.print("From 0 to " + String(level_warn) + " ppm good");
  Serial.print("From " + String(level_warn + 1) + " to " + String(level_danger) + " ppm fair");
  Serial.print("Above " + String(level_danger + 1) + " ppm danger");
#endif

  return RPC_Response("warning", level_warn);
}

// RPC handlers for RPC
RPC_Callback callbacks[] = {
  {"setOffset",         processSetOffset },
  {"calibration",       processCalibration },
  {"userMessage",       processUserMessage },
  {"setLedState",       processSetLedState },
  {"setWarnings",       processSetWarnings },
};
#endif

// This measurments period MEAS_PERIOD in seconds

#define MEAS_PERIOD 20

#ifdef DHT
DHTesp dht;
#endif

#ifdef DHT
// Comfort profile
ComfortState cf;

// Pin number for DHT11 data pin */
int dhtPin = 23;
#endif

/**
  initMeas
  Setup task and timer for repeated measurement
  @return bool
  true if task and timer are started
  false if task or timer couldn't be started
*/
unsigned long start_time; //record the initial millis() to wait for sensor stabilization
#define WARMUPTIME 60

/**
  Task to reads & precess measurements from sensors
*/


MEASUREMENT new_measurement[MAX_KEYS];
void measTask() {
#ifdef DEBUG
  Serial.println("measTask started");
#endif

  display.clearDisplay();
  readMeasurement(new_measurement);
  displayMeasurement(new_measurement);
  publishMeasurement(new_measurement);
  showLeds(new_measurement);
  calibration();
#ifdef DEBUG
  Serial.println("measTask ended");
#endif
}

//Activate leds
void showLeds(MEASUREMENT* new_measurement)
{
  //when enabled==1
  //led 0 lights if co2 value is above danger level (red)
  //led 1 lights if co2 is in between warning and danger levels (yellow)
  //led 2 lights if co2 is below warning level (green)
  //when enabled==0 led is off
  //when enabled==2 led is on

  if (String(new_measurement[0].key) = "co2_ppm") {
    int co2_ppm = new_measurement[0].value;
    Serial.println("level_warn :" + String(level_warn) + " level_danger :" + String(level_danger) + " co2_ppm :" + String(co2_ppm));
    if (co2_ppm <= level_warn) digitalWrite(leds_control[2], true);
    else digitalWrite(leds_control[2], false);
    if (led_state[2] == 0) digitalWrite(leds_control[2], false);
    if (led_state[2] == 2) digitalWrite(leds_control[2], true);

    if ((co2_ppm <= level_danger) && (co2_ppm > level_warn)) digitalWrite(leds_control[1], true);
    else digitalWrite(leds_control[1], false);
    if (led_state[1] == 0) digitalWrite(leds_control[1], false);
    if (led_state[1] == 2) digitalWrite(leds_control[1], true);

    if (co2_ppm > level_danger) digitalWrite(leds_control[0], true);
    else digitalWrite(leds_control[0], false);
    if (led_state[0] == 0) digitalWrite(leds_control[0], false);
    if (led_state[0] == 2) digitalWrite(leds_control[0], true);
  }
}
void calibration() //Calibrate if needed
{
  if (calibrationFlag) {
#ifdef MHZ19_CO2
    //Check if a calibration is needed
    myMHZ19.calibrate();
    myMHZ19.autoCalibration(true);
    // set Range to RANGE_Z19 using a function, see below (disabled as part of calibration)
    setRange(RANGE_Z19);
#endif
#ifdef CM1106_CO2
    //Check if a calibration is needed
    myCM1106.calibrateZero();
    myCM1106.autoCalibration(true, 7);
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
void readMeasurement(MEASUREMENT * measurements)
{
  int co2_ppm; //concertration of CO2
  float co2_t; //Temperature of the sensor (not very accurate as sensor generates heat)
  int keyIndex = 0;

#ifdef MHZ19_CO2

  /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even
    if below background CO2 levels or above range (useful to validate sensor). You can use the
    usual documented command with getCO2(false) */

  co2_ppm = co2Offset + myMHZ19.getCO2(); // Request CO2 (as ppm) unlimimted value, new request
  //co2_ppm = co2Offset + myMHZ19.getCO2(true, true); // Request CO2 (as ppm) unlimimted value, new request
  co2_t = myMHZ19.getTemperature(true, false);   // decimal value, not new request
  //co2_t = myMHZ19.getTempAdjustment();   // adjusted temperature

  measurements[keyIndex].key = String("co2_ppm");
  measurements[keyIndex].value = float(co2_ppm);
  keyIndex++;

  measurements[keyIndex].key = String("co2_t");
  measurements[keyIndex].value = float(co2_t);
  keyIndex++;

  measurements[keyIndex].key = String("co2_raw");
  measurements[keyIndex].value = float(myMHZ19.getCO2Raw());
  keyIndex++;

  double adjustedCO2 = myMHZ19.getCO2Raw();

  adjustedCO2 = 6.60435861e+15 * exp(-8.78661228e-04 * adjustedCO2);      // Exponential equation for Raw & CO2 relationship
  measurements[keyIndex].key = String("co2_adj");
  measurements[keyIndex].value = float(adjustedCO2);
  keyIndex++;

#ifdef CM1106_CO2
  co2_ppm = co2Offset + myCM1106.getCO2(); // Request CO2 (as ppm) unlimimted value, new request
  measurements[keyIndex].key = String("co2_ppm");
  measurements[keyIndex].value = float(co2_ppm);
  keyIndex++;
#endif

#endif
#ifdef DHT
  TempAndHumidity newValues;
  float heatIndex = -99;
  float dewPoint = -99;
  float cr = -99;

  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {

# ifdef DEBUG
    Serial.println("DHT11 error status : " + String(dht.getStatusString()));
# endif
    measurements[keyIndex].key[0] = 0;
    //    strcpy(measurements[keyIndex].key, NO_KEY);
  } else {

    newValues = dht.getTempAndHumidity();

    heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
    dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
    cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

    //record key value pairs on the measurementes array (to be published)
    measurements[keyIndex].key = String("temperature");
    measurements[keyIndex].value = float(newValues.temperature);
    keyIndex++;

    measurements[keyIndex].key = String("humidity");
    measurements[keyIndex].value = float(newValues.humidity);
    keyIndex++;

    measurements[keyIndex].key = String("dewPoint");
    measurements[keyIndex].value = float(dewPoint);
    keyIndex++;

    measurements[keyIndex].key = String("heatIndex");
    measurements[keyIndex].value = float(heatIndex);
    keyIndex++;

    measurements[keyIndex].key = String("comfortRatio");
    measurements[keyIndex].value = float(cr);
    keyIndex++;
  }
#endif
#ifdef BME280

  measurements[keyIndex].key = String("temperature");
  measurements[keyIndex].value = float(bme.readTemperature());
  keyIndex++;

  measurements[keyIndex].key = String("pressure");
  measurements[keyIndex].value = float(bme.readPressure());
  keyIndex++;

  measurements[keyIndex].key = String("humidity");
  measurements[keyIndex].value = float(bme.readHumidity());
  keyIndex++;

  measurements[keyIndex].key = String("altitude");
  measurements[keyIndex].value = float(bme.readAltitude(SEALEVELPRESSURE_HPA));
  keyIndex++;

#endif
  //end with an empty key
  measurements[keyIndex].key = String(NO_KEY);
}

//  Publishes the measurements to the mqtt broker
bool publishMeasurement(MEASUREMENT * mesurements)
{
  Serial.println("Start publishing");
  if (checkConnection()) {
    for (int i = 0; i < MAX_KEYS; i++) {
      if (mesurements[i].key == NO_KEY) break;
      tb.sendTelemetryFloat(mesurements[i].key.c_str(), mesurements[i].value);
#ifdef DEBUG
      Serial.println("key : " + mesurements[i].key + " value : " + String(mesurements[i].value));
#endif
    }
    return true;
  } else return false;
}
//Function to display the measurements of the measuments array

void displayMeasurement(MEASUREMENT * mesurements)
{
#ifdef DISPLAY
  int lines = DISPLAY_LINES - 1; // do not overflow display
  char line[20];
  display.clearDisplay();
  display.setTextSize(2);
  lines /= 2;
  if (MAX_KEYS < lines) lines = MAX_KEYS;
  //display.clearDisplay();
  display.setCursor(0, 0); //top left
  int j = 0;
  for (int i = 0; i < MAX_KEYS; i++) {
    if (mesurements[i].key == NO_KEY || j == lines) break;
    else if (mesurements[i].key == "co2_ppm")
    {
      j++;
      snprintf(line, sizeof(line), "CO2: %-3.0f", mesurements[i].value);
      display.println(line);
    }
    else if (mesurements[i].key == "humidity")
    {
      j++;
      snprintf(line, sizeof(line), "Hum: %-2.0f", mesurements[i].value);
      display.println(line);
    }
    else if (mesurements[i].key == "temperature")
    {
      j++;
      snprintf(line, sizeof(line), "Tem: %-2.1f", mesurements[i].value);
      display.println(line);
    }
  }

  display.setTextSize(1);
  display.setCursor(0, (DISPLAY_LINES - 1) * 8); //last row
  if (userMessage != "") display.println(userMessage);
  else display.println(token);
  display.display();
#endif
}
/*
  void displayMeasurement(MEASUREMENT * mesurements)
  {
  #ifdef DISPLAY
  int lines = DISPLAY_LINES - 1; // do not overflow display

  if (MAX_KEYS < lines) lines = MAX_KEYS;
  //display.clearDisplay();
  display.setCursor(0, 0); //top left
  for (int i = 0; i < lines; i++) {
    if (mesurements[i].key == NO_KEY ) break;
    char line[20];
    snprintf(line, sizeof(line), "%-10.10s: %-3.1f",
             mesurements[i].key.c_str(), mesurements[i].value);
    display.println(line);
  }
  display.setCursor(0, (DISPLAY_LINES - 1) * 8); //last row
  if (userMessage != "") display.println(userMessage);
  else display.println(token);
  display.display();
  #endif
  }
*/

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
  if (last_m == 0) last_m = millis(); //first execution
  if ((current_m - last_m) / 1000 > UPDATE_PERIOD)
  {
#ifdef DEBUG
    Serial.println("Current Type : " + String(SW_TYPE) + " Version : " + String(SW_VERSION));
#endif
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
  Serial.println("mounting FS...");
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        deserializeJson(doc, configFile);
        serializeJson(doc, Serial);
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
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
  Serial.println("saving config");
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

  Serial.println("Starting portal if needed");
  display.println();
  display.println("Please connect to:");
  display.println(token);
  display.display();
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
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println("Esperando medidas");
  display.setTextSize(1);
  display.display();
#endif
  //if you get here you have connected to the WiFi
#ifdef DEBUG
  Serial.println("connected...yeey : )");
#endif
  //Save the credentials just in case we need to reconnect
  strcpy(ap_ssid, wifiManager.getWiFiSSID().c_str());
  strcpy(ap_pass, wifiManager.getWiFiPass().c_str());
  //return configJ;
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
#ifdef DEBUG
    Serial.println("Disconnected from AP");
#endif
    display.drawBitmap(display.width() - 16, 0, cancel_icon16x16, 16, 16, 1);
    display.display();
    WiFi.begin(ap_ssid, ap_pass);
    success = false;
  } else  {
    display.drawBitmap(display.width() - 16, 0, wifi1_icon16x16, 16, 16, 1);
    display.display();
    // Reconnect to ThingsBoard, if needed

#ifdef DEBUG
    Serial.println(token);
#endif
    if (!tb.connected()) {
      display.drawBitmap(display.width() - 16, 16, cancel_icon16x16, 16, 16, 1);
      display.display();
      success = false;
      // Connect to the ThingsBoard
#ifdef DEBUG
      Serial.print("Connecting to : ");
      Serial.print((const char*)configJson["mqtt_server"]);
      Serial.print(" with token: ");
      Serial.println(token);
#endif
      if (!tb.connect(configJson["mqtt_server"], token)) {
#ifdef DEBUG
        Serial.println("Failed to connect to ThingsBoard ");
#endif
        success = false;
      }
#ifdef RPC
#ifdef DEBUG
      Serial.println("Subscribing for RPC...");
#endif

      // Perform a subscription. All consequent data processing will happen in
      // callbacks as denoted by callbacks[] array.
      if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
#ifdef DEBUG
        Serial.println("Failed to subscribe for RPC");
#endif
        success = false;
      }
#ifdef DEBUG
      Serial.println("Subscribe done");
#endif
#endif
    }
  }
#ifdef DISPLAY
  if (success) display.drawBitmap(display.width() - 16, 16, arrow_up_icon16x16, 16, 16, 1);
  else display.drawBitmap(display.width() - 16, 16, cancel_icon16x16, 16, 16, 1);
  display.display();
#endif
  return success;
}

HardwareSerial mySerial(1);     // ESP32 serial port to communicate with sensor
bool tasksEnabled = false;
unsigned long current_time;
unsigned long last_time;
void setup()
{
  //initialize configuration doc with preset values
  configJson["mqtt_server"] = THINGSBOARD_SERVER;
  configJson["mqtt_port"] = THINGSBOARD_PORT;
  configJson["telemetry_topic"] = TELEMETRY_TOPIC;
  configJson["sw_server"] = SW_SERVER;

  start_time = millis(); //power on time
#ifdef DEBUG
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println("Booting");
#endif
  Wire.begin(I2C_SDA, I2C_SCL);
#ifdef DISPLAY
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
#ifdef DEBUG
    Serial.println(F("SSD1306 allocation failed"));
#endif
    for (;;); // Don't proceed, loop forever
  }
  display.setTextSize(3);             // Draw 3X-scale text
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.clearDisplay();
  display.setCursor(13, 0);            // Start at top-left corner
  display.println("CaeliA");
  display.display();
  display.setTextSize(1);             // Normal 1:1 pixel scale
#endif
  InitWiFi();

  //ennable uploading over the air
  //register the server where to check if firmware needs updating
  esp32FOTA.checkURL = String((const char*)configJson["sw_server"]);
  // initialize control pins for leds
  for (int i = 0; i < sizeof(leds_control); i++) pinMode(leds_control[i], OUTPUT);

#ifdef MHZ19_CO2
  mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);   // device to MH-Z19 serial start
  delay(500);
  myMHZ19.begin(mySerial);       // *Important, Pass your Stream reference
  delay(5000);                   // Wait for sensor to stabilize
  myMHZ19.autoCalibration(true);
  setRange(RANGE_Z19);                // set Range 5000 using a function, see below (disabled as part of calibration)
#ifdef DEBUG
  //  Primary Information block
  Serial.println("ProgrammID : ESP32_Z19 ");
  myMHZ19.getVersion(myVersion);
  Serial.print("Firmware Version : ");
  for (byte i = 0; i < 4; i++)
  {
    Serial.print(myVersion[i]);
    if (i == 1)
      Serial.print(".");
  }
  Serial.println("");
  Serial.print("Background CO2 : ");
  Serial.println(myMHZ19.getBackgroundCO2());
  Serial.print("Temperature Cal : ");
  Serial.println(myMHZ19.getTempAdjustment());
  Serial.println("------------------------------ -");
#endif
#endif

#ifdef DHT
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT11);
# ifdef DEBUG
  Serial.println("DHT initiated");
# endif
#endif
#ifdef BME280
  if (! bme.begin(0x77, &Wire)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
#endif

  delay(2000); // Pause for 2 seconds

  //Check if update is needed
  if (esp32FOTA.execHTTPcheck()) esp32FOTA.execOTA();
  //Initiallize measurement tasks


  // Signal end of setup() to tasks
  tasksEnabled = false;
  //wait for warmup
  if (millis() - start_time < WARMUPTIME * 1000)
  {
    Serial.println("Waiting for: " + String((WARMUPTIME * 1000 - (millis() - start_time)) / 1000) + " seconds");
    delay((WARMUPTIME * 1000 - (millis() - start_time)));
  }
  last_time = millis();
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
  updateSW(); //check for sw updates
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
  Serial.println("Setting range..");
  myMHZ19.setRange(range);                        // request new range write
  if ((myMHZ19.errorCode == RESULT_OK) && (myMHZ19.getRange() == range))     //RESULT_OK is an alias from the library,
  {
    Serial.print("Range : ");
    Serial.print(range);
    Serial.println(" successfully applied.");  // Success
  }
  else
  {
    Serial.print("Failed to set Range! Error Code : ");
    Serial.println(myMHZ19.errorCode);          // Get the Error Code value
  }
}
#endif
