/* -------------------------------------------------
  Author: Juan Goicolea juan.goicolea@gmail.com
  
  Version: 0.1

  License: LGPLv3

  Library supporting CM1106 sensors
----------------------------------------------------- */

#include "CM1106.h"
#define DEBUGLIB
#ifdef DEBUGLIB
void printHex(uint8_t num) {
  char hexCar[3];

  sprintf(hexCar, "%02X", num);
  hexCar[3]='\0';
  Serial.print(hexCar);
}
#endif

/*#########################-Commands-##############################*/

byte CM_Commands[6] = {
    0x01, // 0 Read measured result of CO2
    0x10, // 1 Open/ Close ABC and set ABC parameter
    0x03, // 2 Calibrate concentration value of CO2
    0x1F, // 3 Read the serial number of the sensor
    0x1E, // 4 Read software version
    0x10, // 5 ABC close
};

/*#####################-Initiation Functions-#####################*/

void CM1106::begin(Stream &serial) 
{  
    mySerial = &serial;    
    
    /* establish connection */
    verify();

    /* check if successful */
    if (this->errorCode != CM_RESULT_OK) 
    {
        #if defined (ESP32) && (CM1106_ERRORS)
        ESP_LOGE(TAG_CM1106, "Initial communication errorCode recieved");
        #elif CM1106_ERRORS
        Serial.println("!ERROR: Initial communication errorCode recieved");
        #endif 
    }
}


/*########################-Get Functions-##########################*/

int CM1106::getCO2(bool isunLimited, bool force)
{
    unsigned int validRead = 0;
    provisioning(GETCO2);
       
    if (this->errorCode == CM_RESULT_OK )
    {            
        validRead = makeInt(this->storage.responses.GETCO2[3], this->storage.responses.GETCO2[4]); 
    }             
    return validRead;
}

void CM1106::getSerialN(char rSerialN[])
{
    provisioning(GETSN);

    if (this->errorCode == CM_RESULT_OK)
        for (byte i = 0; i < 5; i++)
        {
            rSerialN[i] = char(this->storage.responses.STAT[i + 3]);
        }

    else
        memset(rSerialN, 0, 4);
}

 
void CM1106::getVersion(char rVersion[])
{
    provisioning(GETFIRMWARE);

    if (this->errorCode == CM_RESULT_OK)
        for (byte i = 0; i < 10; i++)
        {
            rVersion[i] = char(this->storage.responses.STAT[i + 3]);
        }

    else
        memset(rVersion, 0, 10);
}



byte CM1106::getLastResponse(byte bytenum)
{
    provisioning(GETCO2);

    if (this->errorCode == CM_RESULT_OK)
        return (this->storage.responses.STAT[bytenum]);

    else
        return 0;
}


void CM1106::autoCalibration(bool isON, byte ABCPeriod)
{
    if(isON) provisioning(ABC, ABCPeriod);
    else provisioning(ABC_off, ABCPeriod);
}

void CM1106::calibrateZero(int rangeCal)
{
    if (rangeCal)
    {

        provisioning(ZEROCAL, rangeCal);
    }

    else
        provisioning(ZEROCAL, 400);
}

/*######################-Utility Functions-########################*/

void CM1106::verify()
{
    unsigned long timeStamp = millis();

    /* construct common command (133) */
    constructCommand(GETCO2);

    write(this->storage.constructedCommand);

    while (read(this->storage.responses.GETCO2, GETCO2) != CM_RESULT_OK)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD)
        {
           #if defined (ESP32) && (CM1106_ERRORS)
            ESP_LOGE(TAG_CM1106, "Failed to verify connection(1) to sensor.");   
            #elif CM1106_ERRORS
            Serial.println("!ERROR: Failed to verify connection(1) to sensor.");
            #endif   

            return;
        }
    }

    /* construct & write last response command (162) */
    constructCommand(GETLASTRESP);
    write(this->storage.constructedCommand);
    
    /* update timeStamp  for next comms iteration */ 
    timeStamp = millis();

    while (read(this->storage.responses.STAT, GETLASTRESP) != CM_RESULT_OK)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD)
        {
            #if defined (ESP32) && (CM1106_ERRORS)
            ESP_LOGE(TAG_CM1106, "Failed to verify connection(2) to sensor.");   
            #elif CM1106_ERRORS
            Serial.println("!ERROR: Failed to verify connection(2) to sensor.");
            #endif
            
            return;
        }
    }      

    /* compare CO2 & temp bytes, command(133), against last response bytes, command (162)*/
    for (byte i = 2; i < 6; i++)
    {
        if (this->storage.responses.GETCO2[i] != this->storage.responses.STAT[i])
        {
            #if defined (ESP32) && (CM1106_ERRORS)
            ESP_LOGE(TAG_CM1106, "Last response is not as expected, verification failed.");   
            #elif CM1106_ERRORS
            Serial.println("!ERROR: Last response is not as expected, verification failed.");
            #endif

            return;
        }
    }
    return;
}


void CM1106::printCommunication(bool isDec, bool isPrintComm)
{
    this->storage.settings._isDec = isDec;
    this->storage.settings.printcomm = isPrintComm;
}

/*######################-Inernal Functions-########################*/

void CM1106::provisioning(Command_Type commandtype, int inData)
{
    /* construct command */
#ifdef DEBUGLIB
    Serial.println("provisioning");
    Serial.println("commandtype: "+String(commandtype)+ " inData: "+String(inData));
#endif
    constructCommand(commandtype, inData);
#ifdef DEBUGLIB
    byte* buffer=this->storage.constructedCommand;
    Serial.print("Command to be sent: ");
    for (int i=0;i<buffer[1]+3;i++) printHex(buffer[i]);
    Serial.println();
#endif
    /* write to serial */
    write(this->storage.constructedCommand);

    /*return response */
    handleResponse(commandtype);

}

void CM1106::constructCommand(Command_Type commandtype, int inData)
{
    /* values for conversions */
    byte High;
    byte Low;

    /* Temporary holder */
    byte asemblecommand[CM1106_DATA_LEN];
#ifdef DEBUGLIB
    Serial.println("constructCommand");
    Serial.println("commandtype: "+String(commandtype)+ " inData: "+String(inData));
#endif

    /* prepare arrays */
    memset(asemblecommand, 0, CM1106_DATA_LEN);
    memset(this->storage.constructedCommand, 0, CM1106_DATA_LEN);

    /* set start byte' */
    asemblecommand[0] = 0x11; //Start byte

    /* set  length of payload */
    asemblecommand[1] = 1; //Length of frame bytes= data length +1 (including CMD+DATA)

    /* set command */
    asemblecommand[2] = CM_Commands[commandtype]; // assign command value

    switch (commandtype)
    {
    case ABC:
        if (this->storage.settings.ABCRepeat == false)
        {
            asemblecommand[1] = 7;
            asemblecommand[3] = 0x64;
            asemblecommand[4] = 0x0; //0x2 to close
            asemblecommand[5] = inData; //Calibration cycle
            asemblecommand[6] = 400/256; //Calibration value
            asemblecommand[7] = 400%256;
            asemblecommand[8] = 0x64;
        }
        break;

    case ABC_off:
        if (this->storage.settings.ABCRepeat == false)
        {
            asemblecommand[1] = 7;
            asemblecommand[3] = 0x64;
            asemblecommand[4] = 0x02; //0x2 to close
            asemblecommand[5] = inData; //Calibration cycle
            asemblecommand[6] = 400/256; //Calibration value
            asemblecommand[7] = 400%256;
            asemblecommand[8] = 0x64;
        }
        break;

    case GETCO2:
        break;

    case ZEROCAL:
        if (inData)
        {
            asemblecommand[1] = 3;
            asemblecommand[3] = inData/256; //Calibration value
            asemblecommand[4] = inData%256;
        } 
        break;

    case GETSN:
        break;

    case GETFIRMWARE:
        break;

    case GETLASTRESP:
        break;
    }


    /* set checksum */
    byte len=asemblecommand[1]+2;
    asemblecommand[len] = getCRC(asemblecommand, len);

#ifdef DEBUGLIB
    Serial.println("Len: "+String(asemblecommand[1]));
    printHex(asemblecommand[1]);
    Serial.println();
    Serial.print("Assembled command: ");
    for (int i=0;i<asemblecommand[1]+3;i++) printHex(asemblecommand[i]);
    Serial.println();
#endif

    /* copy bytes from asemblecommand to constructedCommand */
    memcpy(this->storage.constructedCommand, asemblecommand, len+1);
}

void CM1106::write(byte toSend[])
{
    /* for print communications */
    if (this->storage.settings.printcomm == true)
        printstream(toSend, true, this->errorCode);

    /* transfer to buffer */
    mySerial->write(toSend, CM1106_DATA_LEN); 
 
    /* send */
    mySerial->flush(); 
}

byte CM1106::read(byte inBytes[CM1106_DATA_LEN], Command_Type commandnumber)
{
    /* loop escape */
    unsigned long timeStamp = millis();

    /* prepare memory array with unsigned chars of 0 */
    memset(inBytes, 0, CM1106_DATA_LEN);

    /* prepare errorCode */
    this->errorCode = CM_RESULT_NULL;

    //Wait for the header characters

    while (mySerial->available() < 3)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD) 
        {
            #if defined (ESP32) && (MHZ19_ERRORS) 
            ESP_LOGW(TAG_MHZ19, "Timed out waiting for response");    
            #elif MHZ19_ERRORS
            Serial.println("!Error: Timed out waiting for response");
            #endif  

            this->errorCode = CM_RESULT_TIMEOUT;
            
            /* clear incomplete 9 byte values, limit is finite */
            cleanUp(mySerial->available());

            //return error condition
            return CM_RESULT_TIMEOUT;
        }
    }    
    mySerial->readBytes(inBytes, 3);
    int len=inBytes[1];
    /* wait until we have exactly the expected bytes reply
    this used to be <= 0 but then on very fast controlles such as the ESP only 1 bytes was read
    as the transmission is on a slow 9600 and the system did not wait on the rest...
    */

    while (mySerial->available() < len)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD) 
        {
            #if defined (ESP32) && (MHZ19_ERRORS) 
            ESP_LOGW(TAG_MHZ19, "Timed out waiting for response");    
            #elif MHZ19_ERRORS
            Serial.println("!Error: Timed out waiting for response");
            #endif  

            this->errorCode = CM_RESULT_TIMEOUT;
            
            /* clear incomplete 9 byte values, limit is finite */
            cleanUp(mySerial->available());

            //return error condition
            return CM_RESULT_TIMEOUT;
        }
    }    

    /* response recieved, read buffer */
    mySerial->readBytes(inBytes+3, len);

    if (this->errorCode == CM_RESULT_TIMEOUT)
        return this->errorCode;

    byte crc = getCRC(inBytes,inBytes[1]+1);

    /* CRC error will not overide match error */
    if (inBytes[8] != crc)
        this->errorCode = CM_RESULT_CRC;

    /* construct error code */
    if (inBytes[0] != this->storage.constructedCommand[0] || inBytes[1] != this->storage.constructedCommand[2])
        this->errorCode = CM_RESULT_MATCH;

    /* if error has been assigned */
    if (this->errorCode == CM_RESULT_NULL)
        this->errorCode = CM_RESULT_OK;

    /* print results */
    if (this->storage.settings.printcomm == true)
        printstream(inBytes, false, this->errorCode);

    return this->errorCode;
}

void CM1106::cleanUp(uint8_t cnt)
{
    uint8_t eject = 0;
    for(uint8_t x = 0; x < cnt; x++)
    {
        eject = mySerial->read();
        #if defined (ESP32) && (CM1106_ERRORS) 
        ESP_LOGW(TAG_MHZ19, "Clearing Byte: %d", eject);  
        #elif CM1106_ERRORS
        Serial.print("!Warning: Clearing Byte: "); Serial.println(eject);
        #endif     
    }
}

void CM1106::handleResponse(Command_Type commandtype)
{
    if (this->storage.constructedCommand[2] == CM_Commands[0])      // compare commands byte
        read(this->storage.responses.GETCO2, commandtype);       // returns error number, passes back response and inputs command

    else
        read(this->storage.responses.STAT, commandtype);
}

void CM1106::printstream(byte inBytes[CM1106_DATA_LEN], bool isSent, byte pserrorCode)
{
    if (pserrorCode != CM_RESULT_OK && isSent == false)
    {
        Serial.print("Recieved >> ");
        if (this->storage.settings._isDec)
        {
            Serial.print("DEC: ");
            for (uint8_t i = 0; i < inBytes[i]+3; i++)
            {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        }
        else
        {
            for (uint8_t i = 0; i < inBytes[i]+3; i++)
            {
                Serial.print("0x");
                if (inBytes[i] < 16)
                    Serial.print("0");
                Serial.print(inBytes[i], HEX);
                Serial.print(" ");
            }
        }
        Serial.print("ERROR Code: ");
        Serial.println(pserrorCode);
    }

    else
    {
        isSent ? Serial.print("Sent << ") : Serial.print("Recieved >> ");

        if (this->storage.settings._isDec)
        {
            Serial.print("DEC: ");
            for (uint8_t i = 0; i < inBytes[i]+3; i++)
            {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        }
        else
        {
            for (uint8_t i = 0; i < inBytes[i]+2; i++)
            {
                Serial.print("0x");
                if (inBytes[i] < 16)
                    Serial.print("0");
                Serial.print(inBytes[i], HEX);
                Serial.print(" ");
            }
        }
        Serial.println(" ");
    }
}

byte CM1106::getCRC(byte inBytes[], byte len)
{
    /* as shown in datasheet */
    byte x = 0, CRC = 0;

    for (x = 0; x < len; x++)
    {
        CRC += inBytes[x];
    }

    CRC = 255 - CRC;
    CRC++;

    return CRC;
}


void CM1106::makeByte(int inInt, byte *high, byte *low)
{
    *high = (byte)(inInt / 256);
    *low = (byte)(inInt % 256);

    return;
}

unsigned int CM1106::makeInt(byte high, byte low)
{
    unsigned int calc = ((unsigned int)high * 256) + (unsigned int)low;
 
    return calc;
}