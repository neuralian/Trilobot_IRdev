/* IR tx test 
 *  MGP 21 Jan 2016
 *  32-bit unsigned integers entered in serial terminal 
 *  are re-transmitted as NEC IR messages
 *  
 *  from
 *  
 *  IRrecord (Redux)
   by: Jim Lindblom
   SparkFun Electronics
   date: October 1, 2013


*/
#include <IRremote.h> // Include the IRremote library

int IR_POWER_PIN = 4;
int IR_GND_PIN   = 3;


IRsend irsend; // IR transmit on pin 3

void setup()
{
//  pinMode(IR_POWER_PIN, OUTPUT);
//  pinMode(IR_GND_PIN, OUTPUT);
//  digitalWrite(IR_POWER_PIN, HIGH);
//  digitalWrite(IR_GND_PIN, LOW);
  
  Serial.begin(9600);

}


void loop()
{

 uint32_t msg;  // message is a 32-bit integer
 msg = 0;
 while (Serial.available()>0) {
    char incoming = Serial.read();  // read next char 
    if (incoming=='\n') break;      // break if newline received
//    Serial.print(int(incoming)-48, DEC); Serial.print(", ");
    msg = 10*msg + int(incoming)-48; // 48 is ASCII 0
//    Serial.println(msg, DEC);
    delay(5);
  }


 if (msg>0) {
  irsend.sendNEC(msg, 0x68);
  Serial.println(msg, DEC);
  
 }
    //delay(250); 

}

