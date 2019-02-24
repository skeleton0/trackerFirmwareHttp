#include "Sim7kInterface.h"
#include "Config.h"

Sim7kInterface* sim7k{nullptr};
unsigned long timer{0};

void setup() {
  Serial.begin(4800);
  sim7k = new Sim7kInterface(&Serial);

  sim7k->turnOn();
  sim7k->turnOnGnss();
  sim7k->initHttps();
  sim7k->setHttpsUrl("https://"SERVER_ADDR":"SERVER_PORT);
  
  timer = millis();

  //makes the tracker send an update on start up
  timer += SITTING_UPDATE_FREQUENCY;
}

void loop() {
  sim7k->activateNetwork(APN);

  if (!sim7k->networkIsActive()) {
    return;
  }

  while (handlePositionUpdate()) {
    delay(MOVING_UPDATE_FREQUENCY);
  }
}

bool handlePositionUpdate() {
  if (!sim7k->cachePositionUpdate()) {
      return true;
  }
  
  bool sendUpdate{false};
  
  if (millis() - timer > SITTING_UPDATE_FREQUENCY) {
    writeToLog(F("Sending position due to SITTING_UPDATE_FREQUENCY trigger."));
    sendUpdate = true;
  }
  else if (millis() - timer > MOVING_UPDATE_FREQUENCY && sim7k->positionIsMoving()) {
    writeToLog(F("Sending position due to MOVING_UPDATE_FREQUENCY trigger."));
    sendUpdate = true;
  }

  if (sendUpdate) {
    sim7k->startHttpsConn();
    sim7k->setHttpsContentType();
    sim7k->setHttpsBodyToGnssUpdate(DEVICE_ID);
    
    if (!sim7k->httpsIsConn() || !sim7k->sendHttpsPost("https://"SERVER_ADDR":"SERVER_PORT)) {
      return false;
    }
    
    timer = millis(); //reset timer
  }

  return true;
}

void writeToLog(const __FlashStringHelper* msg) {
  Serial.print(F("Main log - "));
  Serial.println(msg);
}

