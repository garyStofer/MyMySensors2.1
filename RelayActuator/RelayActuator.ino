/**  For MySensor library  V2.1.1 and built under Arduino 1.8.2
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
//#define MY_DEBUG
//#define MY_SPECIAL_DEBUG
//#define MY_DEBUG_VERBOSE_RF24

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_BASE_RADIO_ID 0x00,0x71,0x0f,0x33,0x2a// My own PAN-ID

//#define MY_RF24_CHANNEL  76 -- this is already the default
//#define MY_RF24_DATARATE RF24_250KBPS //this is already the default
//#define MY_RADIO_RFM69

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE
#define MY_NODE_ID 12

#include <MySensors.h>

#define RELAY_1  8  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 1 // Total number of attached relays
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay



void before()
{
	for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
		// Then set relay pins in output mode
		pinMode(pin, OUTPUT);
		// Set relay to last known state (using eeprom storage)
		digitalWrite(pin, RELAY_OFF);
	}
 Serial.println("sensor startup in before");
}

void setup()
{
Serial.println("sensor startup in setup");
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("myRelay", "1.0");
 

	for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
		// Register all sensors to gw (they will be created as child devices)
		present(sensor, S_BINARY);
	}
}


void loop()
{

}

void receive(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
	if (message.type==V_STATUS) {
		// Change relay state
		digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
		// Store state in eeprom
		//saveState(message.sensor, message.getBool());
		// Write some debug info
		Serial.print("Incoming change for sensor:");
		Serial.print(message.sensor);
		Serial.print(", New status: ");
		Serial.println(message.getBool());
	}
}

