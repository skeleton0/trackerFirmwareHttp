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
  while (!sim7k->networkIsActive()) {
    sim7k->activateNetwork(APN);
    delay(1000);
  }

  while (handlePositionUpdate()) {
    delay(MOVING_UPDATE_FREQUENCY);
  }
}

bool handlePositionUpdate() {
  if (!sim7k->cachePositionUpdate()) {
      return true;
  }
  
  if (millis() - timer > SITTING_UPDATE_FREQUENCY) {
    writeToLog(F("Sending position due to SITTING_UPDATE_FREQUENCY trigger."));
  }
  else if (millis() - timer > MOVING_UPDATE_FREQUENCY && sim7k->positionIsMoving()) {
    writeToLog(F("Sending position due to MOVING_UPDATE_FREQUENCY trigger."));
  }
  else
  {
    return true;
  }

  bool readyToSend = sim7k->httpsIsConn() || sim7k->startHttpsConn();

  if (readyToSend && sim7k->setHttpsContentType() && sim7k->setHttpsBodyToGnssUpdate(DEVICE_ID)) {
    //reset timer
    timer = millis();
    return true;
  }

  writeToLog(F("Could not establish HTTPS connection."));
  return false;
}

void writeToLog(const __FlashStringHelper* msg) {
  Serial.print(F("Main log - "));
  Serial.println(msg);
}

