#include "Sim7kInterface.h"
#include "Config.h"

Sim7kInterface* sim7k{nullptr};
Sim7kInterface::ConnectionState state{Sim7kInterface::ConnectionState::MODEM_OFF};
unsigned long timer{0};

void setup() {
  Serial.begin(4800);
  sim7k = new Sim7kInterface(&Serial);
  state = sim7k->queryConnectionState();
  
  timer = millis();

  //makes the tracker send an update on start up
  timer += SITTING_UPDATE_FREQUENCY;
}

void loop() {
  //finite state machine  
  switch (state)
  {
    case Sim7kInterface::ConnectionState::MODEM_OFF:
    sim7k->turnOn();
    sim7k->turnOnGnss();
    state = sim7k->queryConnectionState();
    break;

    case Sim7kInterface::ConnectionState::IP_INITIAL:
    sim7k->cstt(APN);
    state = sim7k->queryConnectionState();
    break;

    case Sim7kInterface::ConnectionState::IP_START:
    sim7k->ciicr();
    state = sim7k->queryConnectionState();
    break;

    case Sim7kInterface::ConnectionState::IP_GPRSACT:
    sim7k->cifsr();
    state = sim7k->queryConnectionState();
    break;

    case Sim7kInterface::ConnectionState::IP_STATUS:
    case Sim7kInterface::ConnectionState::UDP_CLOSED:
    sim7k->cipstart("UDP", SERVER_ADDR, SERVER_PORT);
    state = sim7k->queryConnectionState();
    break;

    case Sim7kInterface::ConnectionState::CONNECT_OK:
    if (!handlePositionUpdate()) {
      state = sim7k->queryConnectionState();
    }
    else {
      //if the tracker is moving and sent an update, by the time we wake up again we'll be due to send another update
      delay(MOVING_UPDATE_FREQUENCY);
    }
    break;

    case Sim7kInterface::ConnectionState::PDP_DEACT:
    sim7k->cipshut();
    state = sim7k->queryConnectionState();
    break;

    default:
    state = sim7k->queryConnectionState();
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
    if (!sim7k->sendGnssUpdate(DEVICE_ID)) {
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

