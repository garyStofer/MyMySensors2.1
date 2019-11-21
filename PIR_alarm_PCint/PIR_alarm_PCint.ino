/*  For MySensor library  V2.1.1 and built under Arduino 1.8.2
 *  
 *  Sketch for a PIR sensor that runs on 2 AA batteries for >2 years, 
 *	See also similar project Wire_Trip_alarm.ino
 *  PCB can be ordered at OSHPCB search for "nrfMysensor" in shared projects, Eagle files at https://github.com/garyStofer/Eagle5_PCB/tree/master/nRF_Mysensor
 */

#include <EEPROM.h>
//#define MY_DEBUG            // turns on Mysensor library debug output
#define MY_BAUD_RATE 57600    // slow down because this node runs at 8Mhz 
#include <MyRadio_NRF24.h>    // Enables NRF24 transport and contains the PAN_ID plus channel number for the radios
// #define MY_NODE_ID 12      // in case you want to make it a specific node ID instead of getting one from the controller
#include <MySensors.h>        // Requires MySensor v2.1.1 or higher 
 
 /*** S E E  N O T E  Below about a required fix to Mysensor library file MyHwAVR.cpp  to allow node to go into indefinite sleep mode ***/
 
// NOTE: Uses PinChange interrupt feature on PortC so that sensor gets triggered on rising AND falling edge of PIR signal. (INT0/1 will only do one or the other on 328p)
// Device is in full power shut down between activations and should take <50ua mostly due to the current consumption of the PIR modul.

// Rising edge sets the alarm state, falling edge clears.  PIR device has trimmer to set the active time
// Note that INT0 and INT1 do not provide interrupt capabilities for both falling and rising edges at the same time (pin change), only one or the other
// and therefore these interrupts can not be used for implementing a sensor that both reports alarm on and alarm off states 

// runs on 328p @3.3V @8mhz internal clock, 4ms startup delay, 1.8V BO -- standard or optiboot loader -- must have correct BL loaded for correct baudrate during programming
// Set Fuses to: 0xFE,0XDE or 0xDA ,0xD2		( Bootloder size as per actual loaded version ) 


/************************* IMPORTANT ************************************ */
//    Must set Brownout voltage to 1.8V and internal clock 8mhz with 4ms startup delay using Atmel Studio programmer with ICE-MKII after loading bootloader.
//    If Brownout voltage is left at default 2.7V battery will not be fully used up.
//    If 8Mhz CPU CLK source startup delay is left at 65 ms the bootloader will not properly connect to reprogram when device was in sleep mode. 
//    A newer version of the Optiboot loader or modifying the defaults in baords.txt for the optiboot loader would also solve this

  
/* MySensor Node application for PIR motion sensor running on Mega328P powered by 2 AAA  Lithium or 2 AA Alkaline cells .
	Standby current consumption with PIR module attached is <50uA.

	This Node will sleep until a pin change is detected on the PIR_SIGNAL_PIN. The Node then reports true or false (alarm ON/OFF) depending on the
	state of the PIR signal output, thus setting and clearing the alarm state.
	The node also reports battery status every BATTERY_REPORT_FREQ time.


  -  Mega328P should be setup to run at 8Mhz internal clock for max reliability and minimum power consumption. See above 
  -  PIR input(s) are using pin change interrupt on port PC. Therefore Analog input A0 (PC0,ADC0) is not available
  -  Standard Arduino power LED and status LED on pin PB5 (SCK) needs to be removed for minimum power consumption.
  -  Input voltage regulator removed, feed B+ to VCC directly. Radio
  -  Standard Radio connection, VCC,D9,D10,MOSI,MISO,SCK,GND for nRF24L
  -  Voltage divider 1M/470K 1% between B+ and BATTERY_V_DIV_GND_PIN for minimum power consumption.
  -  Red LED D8 to GND for loss of RF communication indication. -- Only failure to talk to the parent node is indicated (1st hop).
  -  White or Blue LED VCC to D8 for other indication.
  -  Battery life expectancy with two AAA L92 is about 1 year if the sensor doesn't get triggered constantly.

  -- MUST set Brownout fuse to 1.8V in order to run battery sufficiently down, see above.
  
	See schematis in Eagle files at https://github.com/garyStofer/Eagle5_PCB/tree/master/nRF_Mysensor
	PCB can be ordered at OSHPCB search for "nrfMysensor" in shared projects
*/

#define CHILD_1 1

#define BATTERY_SENSOR_ANALOG_PIN 7  // ADC7, Mega328P device pin 22, this is ANALOG pin 7
#define BATTERY_V_DIV_GND_PIN	  7	   // PD7, Mega328P device pin 11, this is DIGITAL pin 7  	

#define SWITCH_PIN BATTERY_V_DIV_GND_PIN    // This is the same as the BATTERY_V_DIF_GND_PIN.  battery measure voltage divider serves as the pull-up for the switch
#define LED_PIN		 			    8		// PB0, Mega328P device pin 12
#define PIR_SIGNAL_PIN 		  14   		    // PC0, aka ADC0, aka A0, Mega328P device pin 23  -- Do not move this pin -- Pin-Change code below sets up PC0 as input


#define ADC_BITVALUE (1.1f / 1024)    // ADC using 1.1V internal reference
#define ADC_BATTERY_DIV  (3.125f)     // external voltage divider ratio 1M:470K
#define BATTERY_EMPTY (2.6f)
#define BATTERY_FULL  (3.1f)
#define BATTERY_REPORT_FREQ 12*2       // report the battery status only every 24th time for lower power consumption


MyMessage msg(CHILD_1, V_TRIPPED);

// Pin change interrupt to capture the event from the PIR sensor
static  short AlarmInstances = 0; // init so that it sends the battery status on first alarm after startup
ISR(PCINT1_vect)
{

	// nothing to do except to wake up the cpu from sleep

}
//  presentation callback function for Mysensors
void presentation()  //  is called when the controller requests info about the nature of the device 
{

// Send the sketch version information to the Gateway and Controller
  char *sketch = "Battery-PIR-MotionPCint";
  Serial.print("Sending Sketch name ");
  Serial.println(sketch);
  sendSketchInfo(sketch, "1.5");
  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_1, S_MOTION,false,"PIR_SENS");   // gives the sensor a meaningfull name so that domotics can display something upon discovery

}

// before callback funtion for Mysensors
void before()		// executes before the radio starts up
{
  Serial.println("Device Startup.");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // red LED
  pinMode(SWITCH_PIN, INPUT);
  
  // Erasing the EEprom of the chip -- Node will loose it's ID number, Gateway and Controller will furnish a new ID number
  if (digitalRead( SWITCH_PIN ) == 0 )
  {
    Serial.print("Erasing EEPROM\n");
    for (int i = 0; i < 512; i++)
      EEPROM.write(i, 0xff);

    Serial.println("Clearing of EEprom complete.");
  }
  digitalWrite(LED_PIN, LOW);  // white LED
}
void setup()
{

  Serial.print("\nSensor setup() -- Node ID:");
  Serial.println( getNodeId() );
  Serial.flush();


  pinMode(BATTERY_V_DIV_GND_PIN, INPUT);
  digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement

 
  analogReference( INTERNAL);  // on mega328 this is 1.1V
  analogRead(BATTERY_SENSOR_ANALOG_PIN);	// This needs to be done once upfront to setup the ADC before the first reading is taken

  // Setting up the pin change interrupt of Port C
  pinMode(PIR_SIGNAL_PIN, INPUT);       // PC0, aka A0, aka digital pin14, aka device pin23  -- INPUT without pull-up to minimize current 


  
  // enable the pin change interrupt for PC0, aka, A0, aka pin14
  PCMSK1 |= 1 << PCINT8; // enable PCinterupt 8 , aka PC0, aka A0
  PCICR  |= 1 << PCIE1; // enable PCinterupt 1 vector

  pinMode(LED_PIN, INPUT); // leds off
}


void loop()
{
  unsigned int ADC_count;    // for battery reading
  static int BatteryPcnt;  // calc the percent charge of battery --
  float BatteryV;

  pinMode(LED_PIN, INPUT);			// LEDS off

/***  Required fix in library file MyHwAAVR.cpp -- This feature got broken betweenv1.5 and 2.1 of Mysensor lib
This relies on a fix in the MySensor library file "MyHwAVR.cpp" at line 154, allowing 0 ms sleep to do a power down until outside interrupt (pinChange) brings system out of sleep
as documented here:
										int8_t hwSleep(unsigned long ms)
										{
			-->> fix						if (ms>0) {
												// sleep for defined time
												hwInternalSleep(ms);
												return MY_WAKE_UP_BY_TIMER;
			-->> fix						} else {  // Fix for missing feature that allowed (in 1.5.1) to use the pin chnage interrupt with a sleep(0) to completely power down the system
			-->> fix   							// sleep until ext interrupt triggered
			-->> fix							hwPowerDown(SLEEP_FOREVER);
			-->> fix							return 0;
			-->> fix						}
										}
***/


  // The NODE sleeps here until pin-change happens -- This requires a 328P as in a PicoPower device -- Pinchnage interrupt on portC pin is discretly enabled in the setup(). 
  sleep(0);	 
  // when "HERE" the node woke up because of the pin change interrupt
  delay(50); // in case the signal pin bounces
  
  if (digitalRead(PIR_SIGNAL_PIN) )	//Rising edge of PIR status signal. i.e. tripped
  {
    Serial.print("PIR Set\n");
    msg.set(1);    // The status of the PIR Sensor output
  }
  else
  {
    Serial.print("PIR Clear\n");
    msg.set(0);    // The status of the PIR Sensor output
  }

  if (!send(msg, false))    // this returns the state of the HW ACK to the next node (1st hop) -- this only works for single hop networks,
                            // second argument is protocol level ACK, but that requires that the message receive function is implemented and the recived message is checked.
  {
    Serial.print("failed to HW-ack\n");
    digitalWrite(LED_PIN, HIGH);  // set the red LED indicating failure to communicate with parent (gateway or router)
    pinMode(LED_PIN, OUTPUT);    // LEDS on
    delay(300);              // so I can see it
    return;
  }


 
  // send Battery status every Nth time
  if (AlarmInstances++ % BATTERY_REPORT_FREQ==0 ) // 
  {
    Serial.print("Battery Volt,Percent: ");
               
    // setup the voltage divider for the battery measurement
    digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement
    pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);     // activate the voltage divider by pulling the low end to GND

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);	// white LED on for battery load
	delay(500);  // for cap charge up

    ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);
    pinMode(LED_PIN, INPUT);		// LEDS off again

    BatteryV = ADC_count *  ADC_BITVALUE * ADC_BATTERY_DIV;
    BatteryPcnt = 100 * (BatteryV - BATTERY_EMPTY) / (BATTERY_FULL - BATTERY_EMPTY);
	Serial.print(BatteryV);
	Serial.print(", ");
	Serial.println(BatteryPcnt);
	
    if (BatteryPcnt > 100 )
      BatteryPcnt = 100;

    if (BatteryPcnt < 0 )
      BatteryPcnt = 0;
   
    pinMode(BATTERY_V_DIV_GND_PIN, INPUT);      // deactivate the voltage divider by letting it float

    sendBatteryLevel(BatteryPcnt);
    
  }

}






