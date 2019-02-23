#include "Sim7kInterface.h"

#include <Arduino.h>

Sim7kInterface::Sim7kInterface(HardwareSerial* log) :
mLog{log},
mUartStream{10, 11} {
  //init gnss cache
  mGnssCache.mLatitude[0] = '\0';
  mGnssCache.mLongitude[0] = '\0';
  mGnssCache.mTimestamp[0] = '\0';
  mGnssCache.mSpeedOverGround[0] = '\0';
  mGnssCache.mCourseOverGround[0] = '\0';
  
  //init rx buffer with empty string
  mRxCache[0] = '\0';
  
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  
  mUartStream.begin(4800);

  sendCommand("AT");
  if (checkNextResponse("ATOK") || checkLastResponse("OK")) { //ATOK when echo mode is enabled
    writeToLog(F("Modem is initially on."));
    sendInitialSettings();
  }
  else {
    writeToLog(F("Modem is initially off.")); 
  }
  
}

bool Sim7kInterface::turnOn() {
  if (isOn()) {
    return true;
  }

  auto turnOnViaPin = [this](const uint8_t pin) {
    digitalWrite(pin, LOW);
    delay(300);
    digitalWrite(pin, HIGH);
    delay(10000);

    sendCommand("AT");
    if (checkNextResponse("ATOK") || checkLastResponse("OK")) {
      sendInitialSettings();
      return true;
    }

    return false;
  };

  if (turnOnViaPin(6)) {
    writeToLog(F("Turned on modem via pin 6."));
    return true;
  }
  else {
    writeToLog(F("Failed to start modem via pin 6."));
    writeToLog(F("Attempting to start modem with emergency reset via pin 7."));

    if(turnOnViaPin(7)) {
      writeToLog(F("Emergency reset successful."));
      return true;
    }

    writeToLog(F("Emergency reset failed."));
  }
  
  return false;
}

bool Sim7kInterface::turnOff() {
  if (!isOn()) {
    return true;
  }
  
  sendCommand("AT+CPOWD=1");
  return checkNextResponse("NORMAL POWER DOWN");
}

bool Sim7kInterface::isOn() {
  writeToLog(F("Checking if modem is on..."));
  sendCommand("AT");
  return checkNextResponse("OK");
}

bool Sim7kInterface::turnOnGnss() {
  sendCommand("AT+CGNSPWR=1");

  return checkNextResponse("OK");
}

bool Sim7kInterface::cachePositionUpdate() {
  sendCommand("AT+CGNSINF");

  //read GNSS response into mRxCache
  if (!readLineFromUart()) {
    writeToLog(F("Failed to read response from AT+CGNSINF."));
    return false;
  }

  //check if we can separate the first token and it's what we expect from a valid GNSS response
  char* gnssToken = strtok(mRxCache, ",");
  if (!gnssToken || strcmp(gnssToken, "+CGNSINF: 1") != 0) {
    writeToLog(F("Bad GNSS response."));
    return false;
  }

  //check if the GNSS response indicates a fix status
  gnssToken = strtok(nullptr, ",");
  if (!gnssToken || strcmp(gnssToken, "1") != 0) {
    writeToLog(F("GNSS does not have fix status."));
    return false;
  }

  //lambda for copying gnss fields into cache variable
  auto cpyGnssToken = [](char* dst, size_t dstSize) {
    if (char* token = strtok(nullptr, ",")) {
      strncpy(dst, token, dstSize);
      dst[dstSize - 1] = '\0'; //in case something goofd up (i.e. token is longer than dstSize)
      
      return true;
    }

    return false;
  };

  if (cpyGnssToken(mGnssCache.mTimestamp, TIMESTAMP_SIZE) &&
      cpyGnssToken(mGnssCache.mLatitude, LAT_SIZE) &&
      cpyGnssToken(mGnssCache.mLongitude, LON_SIZE) &&
      strtok(nullptr, ",") && //altitude (we don't care about it... for now)
      cpyGnssToken(mGnssCache.mSpeedOverGround, SOG_SIZE) &&
      cpyGnssToken(mGnssCache.mCourseOverGround, COG_SIZE)) {
        
        if (checkNextResponse("OK")) {
          return true;
        }
        else {
          writeToLog(F("Bad GNSS response."));
          return false;
        }
  }

  return false;
}

bool Sim7kInterface::positionIsMoving() {
  if (strcmp(mGnssCache.mSpeedOverGround, "0.00") == 0)
  {
    return false;
  }

  return true;
}

bool Sim7kInterface::cstt(const char* apn) {
  const size_t maxApnLen{50};
  if (strnlen(apn, maxApnLen) == maxApnLen) {
    writeToLog(F("Apn is too long."));
    return false;
  }
  
  char command[60] = "AT+CSTT=\"";
  strcat(command, apn);
  strcat(command, "\"");

  sendCommand(command);
  
  return checkNextResponse("OK");
}

bool Sim7kInterface::ciicr() {
  sendCommand("AT+CIICR");

  return checkNextResponse("OK", 85000);
}

bool Sim7kInterface::cipshut() {
  sendCommand("AT+CIPSHUT");

  return checkNextResponse("SHUT OK");
}

bool Sim7kInterface::cifsr() {
  sendCommand("AT+CIFSR");

  return !checkNextResponse("ERROR");
}

bool Sim7kInterface::cipstart(const char* protocol, const char* address, const char* port) {
  const size_t MAX_PROTO_LEN{4};
  if (strnlen(protocol, MAX_PROTO_LEN) == MAX_PROTO_LEN) {
    writeToLog(F("Argument 'protocol' passed to cipstart is too long."));
    return false;
  }

  const size_t MAX_ADDR_LEN{25};
  if (strnlen(address, MAX_ADDR_LEN) == MAX_ADDR_LEN) {
    writeToLog(F("Argument 'address' passed to cipstart is too long."));
    return false;
  }

  const size_t MAX_PORT_LEN{6};
  if (strnlen(port, MAX_PORT_LEN) == MAX_PORT_LEN) {
    writeToLog(F("Argument 'port' passed to cipstart is too long."));
    return false;
  }

  char command[60] = "AT+CIPSTART=\"";
  strcat(command, protocol);
  strcat(command, "\",\"");
  strcat(command, address);
  strcat(command, "\",");
  strcat(command, port);

  sendCommand(command);

  return checkNextResponse("OK") && checkNextResponse("CONNECT OK", 75000);
}

bool Sim7kInterface::sendGnssUpdate(const char* id) {
  sendCommand("AT+CIPSEND");

  if (checkNextResponse("ERROR", 500)) {
    writeToLog(F("AT+CIPSEND returned error. Probably isn't connected."));
    return false;
  }
  
  //build payload in the format of id,timestamp,lat,lon,sog,cog
  char payload[60] = {};
  strncat(payload, id, 3);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mTimestamp);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mLatitude);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mLongitude);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mSpeedOverGround);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mCourseOverGround);

  writeToLog(F("UDP payload to be sent to server: "));
  writeToLog(payload);

  mUartStream.write(payload);
  mUartStream.write(0x1A); //communicates end of msg to sim7k

  return checkNextResponse("SEND OK", 30000); 
}

Sim7kInterface::ConnectionState Sim7kInterface::queryConnectionState() {
  sendCommand("AT+CIPSTATUS");

  if (checkNextResponse("OK")) {
    if (checkNextResponse("STATE: IP INITIAL")) {
      return ConnectionState::IP_INITIAL;
    }
    else if (checkLastResponse("STATE: IP START")) {
      return ConnectionState::IP_START;
    }
    else if (checkLastResponse("STATE: IP CONFIG")) {
      return ConnectionState::IP_CONFIG;
    }
    else if (checkLastResponse("STATE: IP GPRSACT")) {
      return ConnectionState::IP_GPRSACT;
    }
    else if (checkLastResponse("STATE: IP STATUS")) {
      return ConnectionState::IP_STATUS;
    }
    else if (checkLastResponse("STATE: UDP CONNECTING")) {
      return ConnectionState::UDP_CONNECTING;
    }
    else if (checkLastResponse("STATE: CONNECT OK")) {
      return ConnectionState::CONNECT_OK;
    }
    else if (checkLastResponse("STATE: UDP CLOSING")) {
      return ConnectionState::UDP_CLOSING;
    }
    else if (checkLastResponse("STATE: UDP CLOSED")) {
      return ConnectionState::UDP_CLOSED;
    }
    else if (checkLastResponse("STATE: PDP DEACT")) {
      return ConnectionState::PDP_DEACT;
    }
  }

  return ConnectionState::MODEM_OFF;
}

bool Sim7kInterface::activateNetwork(const char* apn) {
  const size_t maxApnLen{10};
  if (strnlen(apn, maxApnLen) == maxApnLen) {
    writeToLog(F("Apn is too long."));
    return false;
  }

  char command[24] = "AT+CNACT=1,\"";
  strcat(command, apn);
  strcat(command, "\"");

  sendCommand(command);

  return checkNextResponse("+APP PDP: ACTIVE");
}

bool Sim7kInterface::networkIsActive() {
  sendCommand("AT+CNACT?");

  readLineFromUart();

  if (strlen(mRxCache) < 11) {
    return false;
  }

  return mRxCache[8] == '1';
}

bool Sim7kInterface::initHttps() {
  //enable TLS 1.2
  sendCommand("AT+CSSLCFG=\"sslversion\",0,3");

  if (!checkNextResponse("OK")) {
    return false;
  }

  sendCommand("AT+SHCONF=\"BODYLEN\",350");

  if (!checkNextResponse("OK")) {
    return false;
  }

  sendCommand("AT+SHCONF=\"HEADERLEN\",350");

  return checkNextResponse("OK");
}

bool Sim7kInterface::setHttpsUrl(const char* url) {
  if (strnlen(url, URL_LEN_LIMIT) == URL_LEN_LIMIT) {
    writeToLog(F("URL length exceeded the maximum of 64 bytes."));
    return false;
  }

  char command[83] = "AT+SHCONF=\"URL\", \"";
  strcat(command, url);
  strcat(command, "\"");

  sendCommand(command);

  return checkNextResponse("OK");
}

bool Sim7kInterface::startHttpsConn() {
  sendCommand("AT+SHCONN");

  return checkNextResponse("OK", 10000);
}

bool Sim7kInterface::httpsIsConn() {
  sendCommand("AT+SHSTATE?");

  return checkNextResponse("+SHSTATE: 1");
}

bool Sim7kInterface::setBearerApn(const char* apn) {
  const size_t maxApnLen{10};
  if (strnlen(apn, maxApnLen) == maxApnLen) {
    writeToLog(F("Apn is too long."));
    return false;
  }
  
  char command[32] = "AT+SAPBR=3,1,\"APN\",\"";
  strcat(command, apn);
  strcat(command, "\"");

  sendCommand(command);
  
  return checkNextResponse("OK");
}

bool Sim7kInterface::openBearer() {
  sendCommand("AT+SAPBR=1,1");

  return checkNextResponse("OK");
}

Sim7kInterface::BearerStatus Sim7kInterface::getBearerStatus() {
  sendCommand("AT+SAPBR=2,1");
  
  if (!readLineFromUart() || strlen(mRxCache) < 10) {
    return BearerStatus::ERROR;
  }

  BearerStatus status;
  
  //format of response is +SAPBR: <cid>,<Status>,<IP_Addr>
  //the 10th character refers to Status which is in the range [0,3]
  switch (mRxCache[10]) {
    case '0':
    status = BearerStatus::CONNECTING;
    break;
    
    case '1':
    status = BearerStatus::CONNECTED;
    break;
    
    case '2':
    status = BearerStatus::CLOSING;
    break;

    case '3':
    status = BearerStatus::CLOSED;
    break;

    default:
    status = BearerStatus::ERROR;
    break;
  }

  if (!checkNextResponse("OK")) {
    status = BearerStatus::ERROR;
  }

  return status;
}

bool Sim7kInterface::initHttp() {
  sendCommand("AT+HTTPINIT");

  return checkNextResponse("OK");
}

bool Sim7kInterface::setHttpUrl(const char* url) {
  if (strlen(url) > 50) {
    writeToLog(F("URL is too long."));
    return false;
  }

  char command[71] = "AT+HTTPPARA=\"URL\",\"";
  strcat(command, url);
  strcat(command, "\"");

  sendCommand(command);

  return checkNextResponse("OK");
}

bool Sim7kInterface::sendHttpGnssUpdate(const char* id) {
  //build payload in the format of id,timestamp,lat,lon,sog,cog
  char payload[60] = {};
  strncat(payload, id, 3);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mTimestamp);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mLatitude);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mLongitude);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mSpeedOverGround);
  strcat(payload, ",");
  strcat(payload, mGnssCache.mCourseOverGround);

  writeToLog(F("HTTP payload to be sent to server: "));
  writeToLog(payload);

  char command[22];
  const size_t msgLen{strlen(payload)};
  sprintf(command, "AT+HTTPDATA=%d,5000", msgLen);

  sendCommand(command);

  if (!checkNextResponse("DOWNLOAD")) {
    return false;
  }

  mUartStream.write(payload);
  //mUartStream.write(0x1A); //communicates end of msg to sim7k

  if (!checkNextResponse("OK")) {
    return false; 
  }

  sendCommand("AT+HTTPACTION=1");

  return checkNextResponse("OK");
}

void Sim7kInterface::sendCommand(const char* command) {
  flushUart();
  
  writeToLog(F("Sending to modem:"));
  writeToLog(command);
  
  mUartStream.write(command);
  mUartStream.write("\r");
}

//responses from modem are in the form <CR><LF><response><CR><LF>
//if successful, places just <response> in the mRxBuffer
bool Sim7kInterface::readLineFromUart(const uint32_t timeout) { 
  bool foundLineFeed{false};

  for (int i{0}; i < RX_CACHE_SIZE; i++) {
   char nextByte = mUartStream.read();

    //need this loop because arduino will read faster than uart stream transmits
    const uint32_t startTimer = millis();
    while (nextByte == -1) {
      if (millis() - startTimer > timeout) {
        writeToLog(F("readLineFromUart() timed out."));
        mRxCache[0] = '\0';
        return false; 
      }
      
      nextByte = mUartStream.read();
    }
    
    switch (nextByte) {
      case '\n':
      if (foundLineFeed) {
          mRxCache[i] = '\0';
          writeToLog(F("Received response from modem:"));
          writeToLog(mRxCache);
          return true;
      }
      else {
        foundLineFeed = true;
        i--;
      }

      break;
      
      case '\r':
      case '\0':
      i--;
      break;

      default:
      mRxCache[i] = nextByte;
      break;
    }
  }

  writeToLog(F("Uart buffer contains more content than our mRxCache can store"));

  //set buffer to empty string
  mRxCache[0] = '\0';

  //read to buffer failed, so finish flushing the line from the uart stream to avoid leaving a partially consumed response on the buffer
  const uint32_t startTimer = millis();
  char nextByte{0};
  while (!foundLineFeed && nextByte != '\n') {
    if (millis() - startTimer > timeout) {
      writeToLog(F("readLineFromUart() timed out while flushing buffer after read failure."));
      return false;
    }
    
    if (nextByte == '\n') {
      foundLineFeed = true;
    }
    
    nextByte = mUartStream.read();
  }
  
  return false;
}

void Sim7kInterface::flushUart() {
  writeToLog(F("Flushing UART stream."));
  while (mUartStream.available()) {
    readLineFromUart();
  }
}

bool Sim7kInterface::checkNextResponse(const char* expectedResponse, const uint32_t timeout) {
  if (readLineFromUart(timeout)) {
    return strcmp(mRxCache, expectedResponse) == 0;
  }

  return false;
}

bool Sim7kInterface::checkLastResponse(const char* expectedResponse) {
  return strcmp(mRxCache, expectedResponse) == 0;
}

void Sim7kInterface::writeToLog(const __FlashStringHelper* msg) {
  if (mLog) {
    mLog->print(F("Sim7k log - "));
    mLog->println(msg);
  }
}

void Sim7kInterface::writeToLog(const char* msg) {
  if (mLog) {
    mLog->print(F("Sim7k log - "));
    mLog->println(msg); 
  }
}

void Sim7kInterface::sendInitialSettings() {
  //set initial settings
  sendCommand("AT+IPR=4800"); //set baud rate to 4800
  sendCommand("ATE0");        //disable echo mode
  sendCommand("AT+CNMP=38");  //use LTE only
  sendCommand("AT+CMNB=1");   //use CAT-M only
}

