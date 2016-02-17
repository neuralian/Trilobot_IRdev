/*
 * IRremote
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.htm http://arcfn.com
 * Edited by Mitra to add new controller SANYO
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
* LG added by Darryl Smith (based on the JVC protocol)
 */
 
 #include "Arduino.h"
 //#include "Trilobot.h"

#ifndef IRremote_h
#define IRremote_h

 // SparkFun IR Remote:
#define NUM_BUTTONS 9 
// codes transmitted by Sparkfun IR remote = NEC code
// 2 LSbytes only (2MSbytes always the same)
const uint16_t SFIR_POWER_code 		= 0xD827; 
const uint16_t SFIR_A_code 			= 0xF807;
const uint16_t SFIR_B_code 			= 0x7887;
const uint16_t SFIR_C_code 			= 0x58A7;
const uint16_t SFIR_UP_code 		= 0xA05F;
const uint16_t SFIR_DOWN_code 		= 0x00FF;
const uint16_t SFIR_LEFT_code 		= 0x10EF;
const uint16_t SFIR_RIGHT_code 		= 0x807F;
const uint16_t SFIR_CIRCLE_code 	= 0x20DF;
const uint16_t SFIR_REPEAT_code     = 0xFFFF;

// IR receive modes for getButtonpress
// _NO_REPEAT returns code 1x per button press
// _REPEAT returns code 1x per call to getButtonpress while button is held down
#define _SFIR_MODE_NO_REPEAT 0
#define _SFIR_MODE_REPEAT    1

// return values for IR receiver. NO_button is error or no code received
enum SFIR_button {
	NO_buttonpressed, 
	POWER_buttonpressed, 
	A_buttonpressed, 
	B_buttonpressed, 
	C_buttonpressed, 
    UP_buttonpressed, 
	LEFT_buttonpressed, 
	O_buttonpressed, 
	RIGHT_buttonpressed, 
	DOWN_buttonpressed,
	REPEAT_buttonpressed
	};

// IR receiver is on pin 4
#define _Trbot_PIN_IR_RCV   4

// The following are compile-time library options.
// If you change them, recompile the library.
// If DEBUG is defined, a lot of debugging output will be printed during decoding.
// TEST must be defined for the IRtest unittests to work.  It will make some
// methods virtual, which will be slightly slower, which is why it is optional.
// #define DEBUG
// #define TEST

// Results returned from IR decoder
class IRdataclass {
public:
  int code_type; // NEC is codetype 1
  unsigned long value; // 32-bit message
  int team;      
  int player;
  int message;
  int bits; // Number of bits in decoded value
  volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
  int rawlen; // Number of records in rawbuf.
};

// Values for decode_type
#define NEC 1


// Decoded value for NEC when a repeat code is received
#define REPEAT 0xffffffff

// main class for receiving IR
class IRrecv
{
public:
  int repeatMode;  // 0 

  IRrecv(int recvpin);
  IRrecv();
//  void blink13(int blinkflag);
   IRdataclass IRdata;
  
  int getSparkfunRemoteButtonPress();
  long decode(IRdataclass *IRdata);
  void getIRdata(void);  // parses IR message into IRdata
  int getIRmessage(IRdataclass *IRdat);
  void toggleRepeatMode();  // toggle IR receive mode (repeat/no repeat)
 // int decode(decode_results *results);
  void init();
  void resume();
private:



  int compare(unsigned int oldval, unsigned int newval);

  
} ;

// Only used for testing; can remove virtual for shorter code
#ifdef TEST
#define VIRTUAL virtual
#else
#define VIRTUAL
#endif

// class IRsend
// {
// public:
  // IRsend() {}
  // void sendNEC(unsigned long data, int nbits);
  // void sendSony(unsigned long data, int nbits);
  // // Neither Sanyo nor Mitsubishi send is implemented yet
  // //  void sendSanyo(unsigned long data, int nbits);
  // //  void sendMitsubishi(unsigned long data, int nbits);
  // void sendRaw(unsigned int buf[], int len, int hz);
  // void sendRC5(unsigned long data, int nbits);
  // void sendRC6(unsigned long data, int nbits);
  // void sendDISH(unsigned long data, int nbits);
  // void sendSharp(unsigned int address, unsigned int command);
  // void sendSharpRaw(unsigned long data, int nbits);
  // void sendPanasonic(unsigned int address, unsigned long data);
  // void sendJVC(unsigned long data, int nbits, int repeat); // *Note instead of sending the REPEAT constant if you want the JVC repeat signal sent, send the original code value and change the repeat argument from 0 to 1. JVC protocol repeats by skipping the header NOT by sending a separate code value like NEC does.
  // // private:
  // void sendSAMSUNG(unsigned long data, int nbits);
  // void enableIROut(int khz);
  // VIRTUAL void mark(int usec);
  // VIRTUAL void space(int usec);
// }
// ;

// Some useful constants

#define USECPERTICK 50  // microseconds per clock interrupt tick
#define RAWBUF 100 // Length of raw duration buffer

// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
#define MARK_EXCESS 100

#endif
