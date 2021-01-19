/* -------------------------------------------------
  Author: Juan Goicolea juan.goicolea@gmail.com

  Version: 1.5.1

  License: LGPLv3

  Library supporting CM1106 sensors
  ----------------------------------------------------- */

#include "CM1106.h"
//#define CM1106_ERRORS
//#define DEBUGLIB

void printHex(uint8_t num) {
  char hexCar[3];

  sprintf(hexCar, "%02X", num);
  hexCar[3] = '\0';
  Serial.print(hexCar);
}


/*#########################-Commands-##############################*/

byte CM_Commands[6] = {
  0x01, // 0 Read measured result of CO2
  0x10, // 1 Open/ Close ABC and set ABC parameter
  0x03, // 2 Calibrate concentration value of CO2
  0x1F, // 3 Read the serial number of the sensor
  0x1E, // 4 Read software version
  0x10, // 5 ABC close
};
int CM_rlenr[6] = {
  8, // 0 Read measured result of CO2
  4, // 1 Open/ Close ABC and set ABC parameter
  4, // 2 Calibrate concentration value of CO2
  9, // 3 Read the serial number of the sensor
  15, // 4 Read software version
  4, // 5 ABC close
};
int CM_rlens[6] = {
  4, // 0 Read measured result of CO2
  10, // 1 Open/ Close ABC and set ABC parameter
  6, // 2 Calibrate concentration value of CO2
  4, // 3 Read the serial number of the sensor
  4, // 4 Read software version
  10, // 5 ABC close
};
/*#####################-Initiation Functions-#####################*/

void CM1106::begin(Stream &serial)
{
  // establish connection
  mySerial = &serial;
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

byte CM1106::getStatus()
{
  return this->storage.responses.GETCO2[5];
}

void CM1106::getSerialN(byte* rSerialN)
{
  provisioning(GETSN);
  if (this->errorCode == CM_RESULT_CRC) this->errorCode = CM_RESULT_OK; //bug in firmware
  if (this->errorCode == CM_RESULT_OK)
  {
    memcpy(rSerialN, this->storage.responses.STAT + 3, 5);
  }
  else
  {
    memset(rSerialN, 0, 5);
  }
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



byte* CM1106::getLastResponse()
{
  //provisioning(GETCO2);

  if (this->errorCode == CM_RESULT_OK)
    return (this->storage.responses.GETCO2);

  else
    return 0;
}


void CM1106::autoCalibration(bool isON, byte ABCPeriod)
{
  if (isON) provisioning(ABC, ABCPeriod);
  else provisioning(ABC_off, ABCPeriod);
}

void CM1106::calibrate(int rangeCal)
{
  if (rangeCal)
  {

    provisioning(ZEROCAL, rangeCal);
  }

  else
    provisioning(ZEROCAL, 400);
}

/*######################-Utility Functions-########################*/

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
  Serial.println("provisioning...");
  Serial.println("commandtype: " + String(commandtype) + " inData: " + String(inData));
#endif
  constructCommand(commandtype, inData);
#ifdef DEBUGLIB
  byte* buffer = this->storage.constructedCommand;
  Serial.print("Command to be sent: ");
  for (int i = 0; i < CM_rlens[commandtype]; i++)
  {
    printHex(buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
#endif
  /* write to serial */

  write(this->storage.constructedCommand, CM_rlens[commandtype]);

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
  Serial.println("commandtype: " + String(commandtype) + " inData: " + String(inData));
#endif

  /* prepare arrays */
  memset(asemblecommand, 0, CM_rlens[commandtype]);
  memset(this->storage.constructedCommand, 0, CM_rlens[commandtype]);

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
        asemblecommand[6] = 400 / 256; //Calibration value
        asemblecommand[7] = 400 % 256;
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
        asemblecommand[6] = 400 / 256; //Calibration value
        asemblecommand[7] = 400 % 256;
        asemblecommand[8] = 0x64;
      }
      break;

    case GETCO2:
      break;

    case ZEROCAL:
      if (inData)
      {
        asemblecommand[1] = 3;
        asemblecommand[3] = inData / 256; //Calibration value
        asemblecommand[4] = inData % 256;
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
  byte len = asemblecommand[1] + 2;
  asemblecommand[CM_rlens[commandtype] - 1] = getCRC(asemblecommand, CM_rlens[commandtype] - 1);
  //asemblecommand[CM_rlens[commandtype]-1] = getCRC(asemblecommand, len);

#ifdef DEBUGLIB
  Serial.println("Len: " + String(asemblecommand[1]));
  printHex(asemblecommand[1]);
  Serial.println();
  Serial.print("Assembled command: ");
  for (int i = 0; i < CM_rlens[commandtype]; i++)
  {
    printHex(asemblecommand[i]);
    Serial.print(" ");
  }
  Serial.println();
#endif

  /* copy bytes from asemblecommand to constructedCommand */
  memcpy(this->storage.constructedCommand, asemblecommand, CM_rlens[commandtype]);
}

void CM1106::write(byte toSend[], int wlen)
{
  /* for print communications */
  if (this->storage.settings.printcomm == true)
    printstream(toSend, true, this->errorCode);

  /* transfer to buffer */
  mySerial->write(toSend, wlen);

  /* send */
  mySerial->flush();
}

byte CM1106::read(byte inBytes[CM1106_DATA_LEN], Command_Type commandtype)
{
  /* loop escape */

  unsigned long timeStamp = millis();
#ifdef DEBUGLIB
  Serial.println("Entering Read");
  /* prepare memory array with unsigned chars of 0 */
  Serial.println("read command type: " + String(commandtype, HEX));
#endif
  memset(inBytes, 0, CM_rlenr[commandtype]);

  /* prepare errorCode */
  this->errorCode = CM_RESULT_NULL;

  //Wait for the header characters
  //delay(CM1106_DELAY_FOR_ACK);

  // wait until we have exactly the expected bytes reply
  // this used to be <= 0 but then on very fast controlles such as the ESP only 1 bytes was read
  // as the transmission is on a slow 9600 and the system did not wait on the rest...

  mySerial->readBytes(inBytes, CM_rlenr[commandtype]);

  if (millis() - timeStamp >= TIMEOUT_PERIOD)
  {
#ifdef CM1106_ERRORS
    Serial.println("!Error: Timed out waiting for response(1)");
#endif
    this->errorCode = CM_RESULT_TIMEOUT;

    // clear incomplete 9 byte values, limit is finite
    cleanUp(mySerial->available());

    //return error condition
    return CM_RESULT_TIMEOUT;
  }

  // response recieved, read buffer
  //mySerial->readBytes(inBytes, CM_rlenr[commandtype]);
  int len = inBytes[1];
#ifdef DEBUGLIB
  Serial.print("Response: ");
  for (int i = 0; i < CM_rlenr[commandtype]; i++)
  {
    printHex(inBytes[i]);
    Serial.print(" ");
  }
  Serial.println();
#endif
  if (this->errorCode == CM_RESULT_TIMEOUT)
    return this->errorCode;

  byte crc = getCRC(inBytes, CM_rlenr[commandtype] - 1);
#ifdef DEBUGLIB
  Serial.print("crc calculated: ");
  printHex(crc);
  Serial.print(" crc received: ");
  printHex(inBytes[CM_rlenr[commandtype] - 1]);
  Serial.println();
#endif
  /* CRC error will not overide match error */
  if (inBytes[CM_rlenr[commandtype] - 1] != crc)
    this->errorCode = CM_RESULT_CRC;

  /* construct error code */
  if (inBytes[2] != this->storage.constructedCommand[2])
    this->errorCode = CM_RESULT_MATCH;

  /* if error has been assigned */
  if (this->errorCode == CM_RESULT_NULL)
    this->errorCode = CM_RESULT_OK;

  /* print results */
  if (this->storage.settings.printcomm == true)
    printstream(inBytes, false, this->errorCode);
#if DEBUGLIB  
  Serial.println("Error code: "+String(int(this->errorCode)));
#endif
  return this->errorCode;  Serial.println("Error code: "+String(int(this->errorCode)));
}

void CM1106::cleanUp(uint8_t cnt)
{
  uint8_t eject = 0;
  for (uint8_t x = 0; x < cnt; x++)
  {
    eject = mySerial->read();
#ifdef CM1106_ERRORS
    Serial.print("!Warning: Clearing Byte: "); Serial.println(eject);
#endif
  }
}

void CM1106::handleResponse(Command_Type commandtype)
{
#ifdef DEBUGLIB
  Serial.println("Entering handle response");
#endif
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
      for (uint8_t i = 0; i < inBytes[i] + 3; i++)
      {
        Serial.print(inBytes[i]);
        Serial.print(" ");
      }
    }
    else
    {
      for (uint8_t i = 0; i < inBytes[i] + 3; i++)
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
      for (uint8_t i = 0; i < inBytes[i] + 3; i++)
      {
        Serial.print(inBytes[i]);
        Serial.print(" ");
      }
    }
    else
    {
      for (uint8_t i = 0; i < inBytes[i] + 2; i++)
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
  byte crc = 0;
  for (int i = 0; i < len; i++) crc += inBytes[i];
  crc = 255 - crc;
  crc++;
  return crc;
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
