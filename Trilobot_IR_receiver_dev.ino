/* IRrecord (Redux)
   by: Jim Lindblom
   SparkFun Electronics
   date: October 1, 2013

   This sketch uses Ken Shirriff's *awesome* IRremote library:
       https://github.com/shirriff/Arduino-IRremote
   It's a slightly modified version of the IRrecord example that
   comes with that library.

   This sketch uses the IR receiver diode to "record" an IR code.
   Then, when triggered via a button on pin 12, it will transmit
   that IR code out of an attached IR LED on pin 3.

   Hardware setup:
     * The output of an IR Receiver Diode (38 kHz demodulating
       version) should be connected to the Arduino's pin 11.
       * The IR Receiver diode should also be powered off the
         Arduino's 5V and GND rails.
     * The anode (+, longer leg) of an IR LED should be connected to pin 3 of 
       the Arduino. The cathode (-) should be connected to a 330
       Ohm resistor, which connects to ground.
     * A button should be connected to ground on one end, and pin
       12 of the Arduino on the other.

*/
#include <Trilobot.h> // Include the IRremote library

Trilobot bot;

unsigned long ticksNow;
int i;

// setup() initializes serial and the Infrared receiver.
void setup()
{
 bot.init();
   Serial.begin(9600);
   i=0;
}

void loop()
{
	if (bot.getIRmessage()) {
		Serial.println(" ");
		Serial.print("Command: "); Serial.print(bot.IRmsg.command);Serial.print(", ");
		Serial.print("Team: "); Serial.print(bot.IRmsg.team);Serial.print(", ");
		Serial.print("Player: "); Serial.print(bot.IRmsg.player);Serial.print(", ");
		Serial.print("Message: ");Serial.println(bot.IRmsg.message);
	}
	//else Serial.println(++i);
	
	//i = bot.getSparkfunRemoteButtonPress();
	//Serial.println(i);
	delay(250);
	
}