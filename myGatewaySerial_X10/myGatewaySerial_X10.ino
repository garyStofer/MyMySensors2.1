/** For MySensor library  V2.1.1 and built under Arduino 1.8.2
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
* DESCRIPTION
* The ArduinoGateway prints data received from sensors on the serial link.
* The gateway accepts input on seral which will be sent out on radio network.
*
* The GW code is designed for Arduino Nano 328p / 16MHz
*
* Wire connections (OPTIONAL):
* - Inclusion button should be connected between digital pin 3 and GND
* - RX/TX/ERR leds need to be connected between +5V (anode) and digital pin 6/5/4 with resistor 270-330R in a series
*
* LEDs (OPTIONAL):
* - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs
* - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
* - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
* - ERR (red) - fast blink on error during transmission error or recieve crc error
*
*/

// Enable debug prints to serial monitor
//#define MY_DEBUG
//#define MY_SPECIAL_DEBUG
//#define MY_DEBUG_VERBOSE_RF24


// Enable and select radio type attached
#include "MyRadio_NRF24.h"

// Set LOW transmit power level as default, if you have an amplified NRF-module and
// power your radio separately with a good regulator you can turn up PA level.
//#define MY_RF24_PA_LEVEL RF24_PA_HIGH

// Enable serial gateway
#define MY_GATEWAY_SERIAL

// Define a lower baud rate for Arduino's running on 8 MHz (Arduino Pro Mini 3.3V & SenseBender)
#if F_CPU == 8000000L
#define MY_BAUD_RATE 38400
#endif

// Enable inclusion mode
// #define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE

// Inverses behavior of inclusion button (if using external pullup)
//#define MY_INCLUSION_BUTTON_EXTERNAL_PULLUP

// Set inclusion mode duration (in seconds)
//#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Inverses the behavior of leds
//#define MY_WITH_LEDS_BLINKING_INVERSE

// Flash leds on rx/tx/err
// Uncomment to override default HW configurations
#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  5  // the PCB, on board LED

#include <MySensors.h>

// X10 remote control defines
#define CHILD_X10_1       1 // starting at 1
#define MyX10_HOUSE       0 // House code  A -- hardcoded for now
#define X10_PIN           3 // modulation on/off keying pin for 310Mhz transmitter
// bit times as per NEC IR remote control protocol
#define NEC_HDR_SPACE   (8*NEC_BIT_MARK)
#define NEC_HDR_MARK    (16*NEC_BIT_MARK)
#define NEC_BIT_MARK     560
#define NEC_RPT_SPACE   2250
#define MARK 1
#define SPACE 0

void before()
{
}


// Modified usec delay for X10 transmission
void MyDelay_usec(unsigned long uSecs) 
{
// note: micros() might skipp a count if interrupts get in between so we can't just wait for == endcount
// on 16mhz clocked system, micros increments in 4 us steps.
  if (uSecs > 4) 
  {
    unsigned long now = micros();
    unsigned long end = now + uSecs-4;
	
	if (end < now)	// wrapped around
	{
		while ( micros() >= end ) {} //  wait until wrap around
	}
   	
	while ( micros() <= end ) {} //  wait until it hits end 

 } 
}

// controls the pin and timing for the on/off keying of the 310mhz transmitter -- HIGH == TX on
void X10_out ( bool mark, unsigned int time)
{
  digitalWrite(X10_PIN, (mark) ? HIGH :LOW); 
  
  if (time > 0) 
    MyDelay_usec(time);
}

// The lookup table for house codes as they are not sequential.
const static byte HouseCode[] ={6,14,2,10,1,9,5,13,7,15,3,11,0,8,4,12};  // A=0, B=1,....

//Device numbers do not map directly to bits in the command word. Command bit 0 Indicates Dim/Bright, bit 2 controls On/OFF
// Device# bit, (weight) :  Commnad byte bit, (weight)
//		0,(0x1)	: 	3,(0x8)
// 		1,(0x2)	:	4,(0x10)
//		2,(0x4)	:	1,(0x2)
//		3,(0x8)	:	5,(0x20)

  


// Transmits a complete 32 bit burst according to NEC IR control protocl
void X10_transmit( byte house, byte device_no, bool ON_off)
{
  house = HouseCode[house&0xf]; // Looks up sequential house numbers to the nonsequential codes used by X10
  device_no &=0xf;				//max of 16 devices
  
  unsigned short housebits = house |(~house) << 8;
  
  unsigned short device =  ((device_no << 3) &0x18); 	// get bits 3 and 4 in place 
  
  if (device_no & 0x4 )		// fill in out of order device bit 2
	device |= 0x2;
		
  if (device_no & 0x8 )		// fill in out of order device bit 3
	device |= 0x20;
    
  device |=  (ON_off ? 0:0x4);  // Fill in the On/Off bit
  
  unsigned short devicebits = device | ((~device) << 8UL);
  unsigned long X10_bits = housebits | ((unsigned long)devicebits << 16UL); 

  // Header as per NEC IR spec
  X10_out(MARK,NEC_HDR_MARK);
  X10_out(SPACE,NEC_HDR_SPACE);

  //transmit 32 bits LSB first
  for (unsigned long mask = 1;  mask;  mask <<= 1 )
  {
    if (X10_bits & mask) 
    {
      X10_out(MARK,NEC_BIT_MARK);
      X10_out(SPACE,3*NEC_BIT_MARK);  // long space
    } 
    else 
    {
      X10_out(MARK,NEC_BIT_MARK);
      X10_out(SPACE,NEC_BIT_MARK); // short space
    }

  }

     // Footer s per NEC IR spec
  X10_out(MARK,NEC_BIT_MARK);
  X10_out(SPACE,0);  
}
 
void setup()
{
	// Setup locally attached sensors
  pinMode(X10_PIN, OUTPUT);
}

void presentation()
{
	// Present 12 locally attached X10 actuators -- max is 16
  sendSketchInfo("mySensorsGatewayWithX10", "2.11");
  present(CHILD_X10_1, S_BINARY);
  present(CHILD_X10_1+1, S_BINARY);
  present(CHILD_X10_1+2, S_BINARY);
  present(CHILD_X10_1+3, S_BINARY);
  present(CHILD_X10_1+4, S_BINARY);
  present(CHILD_X10_1+5, S_BINARY);
  present(CHILD_X10_1+6, S_BINARY);
  present(CHILD_X10_1+7, S_BINARY);
  present(CHILD_X10_1+8, S_BINARY);
  present(CHILD_X10_1+9, S_BINARY);
  present(CHILD_X10_1+10, S_BINARY);
  present(CHILD_X10_1+11, S_BINARY);

}
void receive(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
	if (message.type==V_STATUS) 
	{
		// Change relay state
		if (message.sensor == CHILD_X10_1) 
			X10_transmit( MyX10_HOUSE, 0, message.getBool());
         
		if (message.sensor == CHILD_X10_1+1) 
			X10_transmit( MyX10_HOUSE, 1, message.getBool());
   
		if (message.sensor == CHILD_X10_1+2) 
			 X10_transmit( MyX10_HOUSE, 2, message.getBool());
       
		if (message.sensor == CHILD_X10_1+3) 
			 X10_transmit( MyX10_HOUSE, 3, message.getBool());

		if (message.sensor == CHILD_X10_1+4) 
			X10_transmit( MyX10_HOUSE, 4, message.getBool());
         
		if (message.sensor == CHILD_X10_1+5) 
         X10_transmit( MyX10_HOUSE, 5, message.getBool());
   
		if (message.sensor == CHILD_X10_1+6) 
			X10_transmit( MyX10_HOUSE, 6, message.getBool());
       
		if (message.sensor == CHILD_X10_1+7) 
			X10_transmit( MyX10_HOUSE, 7, message.getBool());
			
		if (message.sensor == CHILD_X10_1+8) 
			X10_transmit( MyX10_HOUSE, 8, message.getBool());
			
		if (message.sensor == CHILD_X10_1+9) 
			X10_transmit( MyX10_HOUSE, 9, message.getBool());
			
		if (message.sensor == CHILD_X10_1+10) 
			X10_transmit( MyX10_HOUSE, 10, message.getBool());
		
		if (message.sensor == CHILD_X10_1+11) 
			X10_transmit( MyX10_HOUSE, 11, message.getBool());
	}
}

void loop()
{
	// Send locally attached sensor data here

}
