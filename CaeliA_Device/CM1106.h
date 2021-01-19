/* -------------------------------------------------
  Author: Juan Goicolea juan.goicolea@gmail.com
  
  Version: 1.5.1

  License: LGPLv3

  Library supporting CM1106 sensors
-----------------------------------------------------*/

#ifndef CM1106_H
#define CM1106_H
#endif

#include <Arduino.h>

#ifdef ESP32
#include "esp32-hal-log.h"
#endif

#define CM1106_ERRORS 1		   // Set to 0 to disable error prints

#define TEMP_ADJUST 38			// This is the value used to adjust the temeperature.
								// Older datsheets use 40, however is likely incorrect.

// time out period for response 
#define TIMEOUT_PERIOD 10000     // (ms)

// For range mode,  
#define DEFAULT_RANGE 2000     // CM1106 works best in this range

// data sequence length 
#define CM1106_DATA_LEN 16 

// enum alias for error code defintions 


enum CM_ERRORCODE
{
	CM_RESULT_NULL = 0,
	CM_RESULT_OK = 1,
	CM_RESULT_TIMEOUT = 2,
	CM_RESULT_MATCH = 3,
	CM_RESULT_CRC = 4,
	CM_RESULT_FILTER = 5
};
#define CM1106_DELAY_FOR_ACK 500

 // Status
#define CM1106_STATUS_PREHEATING 0x0
#define CM1106_STATUS_NORMAL_OPERATION 0x1
#define CM1106_STATUS_OPERATING_TROUBLE 0x2
#define CM1106_STATUS_OUT_OF_FS 0x3
#define CM1106_STATUS_NON_CALIBRATED 0x5

class CM1106
{
  public:
	// ###########################-Variables-##########################

	// Holds last recieved errorcode from recieveResponse()
	byte errorCode;

	// #####################-Initiation Functions-#####################

	// essential begin 
	void begin(Stream &stream);    

	// ########################-Set Functions-##########################

	// Sets Range to desired value
	//void setRange(int range = 2000);

 
	// ########################-Get Functions-##########################

	// request CO2 value
	int getCO2(bool isunLimited = true, bool force = true);
 
  // Get status after a CO2 read. Can be used to see if sensor is still preheating
  byte getStatus();


	// returns CM1106 SN as an array of 5 bytes to encode a 20  digit SN
	void getSerialN(byte* rSerialN);

	// returns CM1106 sw version as an ascii chain
	void getVersion(char rVersion[]);

	// returns last recorded response from device using command 162
	byte* getLastResponse();

	// ######################-Utility Functions-########################

	// disables calibration or sets ABCPeriod 
	void autoCalibration(bool isON = true, byte ABCPeriod = 24);

  /*  Calibrate Backwards compatability */
  void inline calibrateZero(int rangeCal = 400){ calibrate(rangeCal); };

  // Calibrates "Zero" (Note: Zero refers to 400ppm for this sensor)
  void calibrate(int rangeCal = 400);

	// use to show communication between CM1106 and  Device 
	void printCommunication(bool isDec = true, bool isPrintComm = true);

  private:
	// ###########################-Variables-##########################
     
	// pointer for Stream class to accept reference for hardware and software ports 
  Stream* mySerial; 

  // alias for command types 
	typedef enum COMMAND_TYPE
	{
		
		GETCO2 = 0,		    // 1 Read measured result of CO2
		ABC = 1,		  	// 2 Open/ Close ABC and set ABC parameter
		ZEROCAL = 2,        // 3 Calibrate concentration value of CO2
		GETSN = 3,	        // 4 Read the serial number of the sensor
		GETFIRMWARE = 4,	// 5 Read software version
		ABC_off =5,			// 7 Close ABC
		GETLASTRESP = 6		// 6 Read last response (not used)
	} Command_Type;


	// Memory Pool 
	struct mempool
	{
		struct config
		{
			bool ABCRepeat = false;  					// A flag which represents whether autocalibration ABC period was checked
			bool filterMode = false; 					// Flag set by setFilter() to signify is "filter mode" was made active
			bool filterCleared = true;  				// Additional flag set by setFiler() to store which mode was selected			
			bool printcomm = false; 				    // Communication print options
			bool _isDec = true;							// Holds preferance for communication printing
		} settings;

		byte constructedCommand[CM1106_DATA_LEN];				    	// holder for new commands which are to be sent

		struct indata
		{
			byte GETCO2[CM1106_DATA_LEN];		// Holds command response to getCO2
			byte STAT[CM1106_DATA_LEN];		    // Holds other command responses 
		} responses;

	} storage;

	// ######################-Inernal Functions-########################

	// Coordinates  sending, constructing and recieving commands 
	void provisioning(Command_Type commandtype, int inData = 0);

	// Constructs commands using command array and entered values 
	void constructCommand(Command_Type commandtype, int inData = 0);

	// generates a checksum for sending and verifying incoming data 
	byte getCRC(byte inBytes[], byte len);

	// Sends commands to the sensor 
	void write(byte toSend[], int wlen);

	// Call retrieveData to retrieve values from the sensor and check return code 
	byte read(byte inBytes[9], Command_Type commandnumber);

	// Assigns response to the correct communcation arrays 
	void handleResponse(Command_Type commandtype);

	// prints sending / recieving messages if enabled 
	void printstream(byte inbytes[9], bool isSent, byte pserrorCode);

	// converts integers to bytes according to /256 and %256 
	void makeByte(int inInt, byte *high, byte *low);

	// converts bytes to integers according to *256 and + value 

	unsigned int makeInt(byte high, byte low);

	void cleanUp(uint8_t cnt);

};
//#endif
