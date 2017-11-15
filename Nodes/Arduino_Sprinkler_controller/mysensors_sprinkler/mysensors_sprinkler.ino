/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * Example sketch showing how to control physical relays.
 * This example will remember relay state after power failure.
 * http://www.mysensors.org/build/relay
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE

#include <MySensors.h>

#define RELAY_1  5  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 4 // Total number of attached relays
#define RELAY_ON 0  // GPIO value to write to turn on attached relay
#define RELAY_OFF 1 // GPIO value to write to turn off attached relay

unsigned const long MAX_OPEN_TIME = 600000;

MyMessage msgs[NUMBER_OF_RELAYS];
bool initialValueSent = false;
long onTimeMillis[NUMBER_OF_RELAYS];
void setup()
{
  for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
    // Then set relay pins in output mode
    pinMode(pin, OUTPUT);
    digitalWrite(pin, RELAY_OFF);
    msgs[sensor-1] = MyMessage(sensor, V_STATUS);
  }
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Sprinkler", "1.0");

	for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
		// Register all sensors to gw (they will be created as child devices)
		present(sensor, S_BINARY);
	}
}


void loop()
{  
  if (!initialValueSent) {
    Serial.println("Sending initial value");
    for (int sensor=1; sensor<=NUMBER_OF_RELAYS; sensor++) {
      // Set relay to last known state (using eeprom storage)
      send(msgs[sensor-1].set(onTimeMillis[sensor-1]!=0));
    }
    Serial.println("Requesting initial value from controller");
    for (int sensor=1; sensor<=NUMBER_OF_RELAYS; sensor++) {
      request(sensor, V_STATUS);  
    }
    wait(2000, C_SET, V_STATUS);
  }
  runTimeoutCheck();
}

void receive(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
	if (message.type==V_STATUS) {
    if (!initialValueSent) {
      initialValueSent = true;
    }    
		// Change relay state
		digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
    if (message.getBool()) {
      onTimeMillis[message.sensor-1] = millis();
    } else {
      onTimeMillis[message.sensor-1] = 0;
    }
		// Write some debug info
		Serial.print("Incoming change for sensor:");
		Serial.print(message.sensor);
		Serial.print(", New status: ");
		Serial.println(message.getBool());
    send(msgs[message.sensor-1].set(message.getBool()));
	}
}

void runTimeoutCheck() {
  for (byte i=0; i < NUMBER_OF_RELAYS; i++) {
    if (onTimeMillis[i] > 0 && (millis() > onTimeMillis[i] + MAX_OPEN_TIME)) {
      Serial.print("Autoclosing valve ");
      Serial.println(i);
      digitalWrite(i+RELAY_1, RELAY_OFF);
      send(msgs[i].set(false));
      onTimeMillis[i] = 0;
    }
  }
}

