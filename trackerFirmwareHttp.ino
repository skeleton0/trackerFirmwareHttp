#include "Sim7kInterface.h"
#include "Config.h"

Sim7kInterface* sim7k{nullptr};
Sim7kInterface::BearerStatus status{Sim7kInterface::BearerStatus::ERROR};
unsigned long timer{0};

void setup() {
  Serial.begin(4800);
  sim7k = new Sim7kInterface(&Serial);
  status = sim7k->getBearerStatus();
  
  timer = millis();

  //makes the tracker send an update on start up
  timer += SITTING_UPDATE_FREQUENCY;
}

void loop() { 
  switch (status)
  { 
    case Sim7kInterface::BearerStatus::ERROR:
    sim7k->turnOff();
    delay(1000);
    sim7k->turnOn();
    sim7k->turnOnGnss();
    status = sim7k->getBearerStatus();
    break;

    case Sim7kInterface::BearerStatus::CLOSED:
    if (!sim7k->setBearerApn(APN) || !sim7k->openBearer() || 
        !sim7k->initHttp() || !sim7k->setHttpUrl("http://"SERVER_ADDR":"SERVER_PORT)) {
      status = Sim7kInterface::BearerStatus::ERROR;
    } else {
      status = sim7k->getBearerStatus();
    }
    break;

    case Sim7kInterface::BearerStatus::CONNECTED:
    if (!handlePositionUpdate()) {
      status = sim7k->getBearerStatus();
    } else {
      delay(MOVING_UPDATE_FREQUENCY);
    }
    break;
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
    if (!sim7k->sendHttpGnssUpdate(DEVICE_ID)) {
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

