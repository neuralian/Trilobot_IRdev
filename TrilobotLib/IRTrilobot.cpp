/*
 * IRremote
 * Version 0.11 August, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * Modified by Paul Stoffregen <paul@pjrc.com> to support other boards and timers
 * Modified  by Mitra Ardron <mitra@mitra.biz> 
 * adapted for Bristlebotics Trilobots by Mike Paulin <neuralian@gmail.com> 2015
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 * LG added by Darryl Smith (based on the JVC protocol)
 */

#include "IRTrilobot.h"
#include "IRTrilobotInt.h"

// Provides ISR
#include <avr/interrupt.h>

volatile irparams_t irparams;

// These versions of MATCH, MATCH_MARK, and MATCH_SPACE are only for debugging.
// To use them, set DEBUG in IRremoteInt.h
// Normally macros are used for efficiency
#ifdef DEBUG
int MATCH(int measured, int desired) {
  Serial.print("Testing: ");
  Serial.print(TICKS_LOW(desired), DEC);
  Serial.print(" <= ");
  Serial.print(measured, DEC);
  Serial.print(" <= ");
  Serial.println(TICKS_HIGH(desired), DEC);
  return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);
}

int MATCH_MARK(int measured_ticks, int desired_us) {
  Serial.print("Testing mark ");
  Serial.print(measured_ticks * USECPERTICK, DEC);
  Serial.print(" vs ");
  Serial.print(desired_us, DEC);
  Serial.print(": ");
  Serial.print(TICKS_LOW(desired_us + MARK_EXCESS), DEC);
  Serial.print(" <= ");
  Serial.print(measured_ticks, DEC);
  Serial.print(" <= ");
  Serial.println(TICKS_HIGH(desired_us + MARK_EXCESS), DEC);
  return measured_ticks >= TICKS_LOW(desired_us + MARK_EXCESS) && measured_ticks <= TICKS_HIGH(desired_us + MARK_EXCESS);
}

int MATCH_SPACE(int measured_ticks, int desired_us) {
  Serial.print("Testing space ");
  Serial.print(measured_ticks * USECPERTICK, DEC);
  Serial.print(" vs ");
  Serial.print(desired_us, DEC);
  Serial.print(": ");
  Serial.print(TICKS_LOW(desired_us - MARK_EXCESS), DEC);
  Serial.print(" <= ");
  Serial.print(measured_ticks, DEC);
  Serial.print(" <= ");
  Serial.println(TICKS_HIGH(desired_us - MARK_EXCESS), DEC);
  return measured_ticks >= TICKS_LOW(desired_us - MARK_EXCESS) && measured_ticks <= TICKS_HIGH(desired_us - MARK_EXCESS);
}
#else
int MATCH(int measured, int desired) {return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);}
int MATCH_MARK(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us + MARK_EXCESS));}
int MATCH_SPACE(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us - MARK_EXCESS));}
// Debugging versions are in IRremote.cpp
#endif

IRrecv::IRrecv(int recvpin)
{
  irparams.recvpin = recvpin;
  irparams.blinkflag = 0;
}

IRrecv::IRrecv()
{
  irparams.recvpin = _Trbot_PIN_IR_RCV;
}

// initialization
void IRrecv::init() {
  cli();
  // setup pulse clock timer interrupt
  //Prescale /8 (16M/8 = 0.5 microseconds per tick)
  // Therefore, the timer interval can range from 0.5 to 128 microseconds
  // depending on the reset value (255 to 0)
  TIMER_CONFIG_NORMAL();

  //Timer2 Overflow Interrupt Enable
  TIMER_ENABLE_INTR;

  TIMER_RESET;

  sei();  // enable interrupts

  // initialize state machine variables
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;

  // set pin modes
  pinMode(irparams.recvpin, INPUT);
  
  // receive mode
  repeatMode = _SFIR_MODE_REPEAT;
}


// interrupt code to collect raw data.
// Widths of alternating SPACE, MARK are recorded in rawbuf.
// Recorded in ticks of 50 microseconds.
// rawlen counts the number of entries recorded so far.
// First entry is the SPACE between transmissions.
// As soon as a SPACE gets long, ready is set, state switches to IDLE, timing of SPACE continues.
// As soon as first MARK arrives, gap width is recorded, ready is cleared, and new logging starts
ISR(TIMER_INTR_NAME)
{
  TIMER_RESET;

  uint8_t irdata = (uint8_t)digitalRead(irparams.recvpin);

  irparams.timer++; // One more 50us tick
  if (irparams.rawlen >= RAWBUF) {
    // Buffer overflow
    irparams.rcvstate = STATE_STOP;
  }
  switch(irparams.rcvstate) {
  case STATE_IDLE: // In the middle of a gap
    if (irdata == MARK) {
      if (irparams.timer < GAP_TICKS) {
        // Not big enough to be a gap.
        irparams.timer = 0;
      } 
      else {
        // gap just ended, record duration and start recording transmission
        irparams.rawlen = 0;
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer = 0;
        irparams.rcvstate = STATE_MARK;
      }
    }
    break;
  case STATE_MARK: // timing MARK
    if (irdata == SPACE) {   // MARK ended, record time
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer = 0;
      irparams.rcvstate = STATE_SPACE;
    }
    break;
  case STATE_SPACE: // timing SPACE
    if (irdata == MARK) { // SPACE just ended, record it
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer = 0;
      irparams.rcvstate = STATE_MARK;
    } 
    else { // SPACE
      if (irparams.timer > GAP_TICKS) {
        // big SPACE, indicates gap between codes
        // Mark current code as ready for processing
        // Switch to STOP
        // Don't reset timer; keep counting space width
        irparams.rcvstate = STATE_STOP;
      } 
    }
    break;
  case STATE_STOP: // waiting, measuring gap
    if (irdata == MARK) { // reset gap timer
      irparams.timer = 0;
    }
    break;
  }

}

void IRrecv::resume() {
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;
}

// decode message from Trilobot IR controller
// nb IRdata is a member of IRTrilobot,  IRDat is a member of Trilobot
int IRrecv::getIRmessage(IRdataclass *IRdat){
	
	uint32_t value = (uint32_t)0;
	int OK = 0;

	  IRdata.rawbuf = irparams.rawbuf;
	  IRdata.rawlen = irparams.rawlen;
	  if (irparams.rcvstate != STATE_STOP) return(0);
	
	if (decode(&IRdata)) {
	  value = IRdata.value;
	  IRdat->team     = (int)(value>>24);                // team number in top byte
	  IRdat->player   = (int)((value>>16) & 0x00FF);     // player number in 3rd byte
	  IRdat->message  = (int) (value & 0x0000FFFF);      // message in 2 lower bytes
	  OK = 1;
	}
	resume();  // restart IR receiver

	return(OK);// returns 1 if a value was received
}

// get button code from IR remote
// returns 0 if no code received
int IRrecv::getSparkfunRemoteButtonPress(){  

  int theCode = 0;
  
  SFIR_button theButton = NO_buttonpressed;

  static SFIR_button theLastButton = NO_buttonpressed;
  
  IRdata.rawbuf = irparams.rawbuf;
  IRdata.rawlen = irparams.rawlen;
  if (irparams.rcvstate != STATE_STOP) return(0);
  if (decode(&IRdata)) theCode = IRdata.value & 0xFFFF;
  
  switch(theCode) {
    case SFIR_POWER_code:
	    theButton = POWER_buttonpressed;
		break;
	case SFIR_A_code:
	    theButton = A_buttonpressed;
		break;	  
	case SFIR_B_code:
	    theButton = B_buttonpressed;
		break;	  
	case SFIR_C_code:
	    theButton = C_buttonpressed;
		break;	  
	case SFIR_UP_code:
	    theButton = UP_buttonpressed;
		break;	  
	case SFIR_LEFT_code:
	    theButton = LEFT_buttonpressed;
		break;	  
	case SFIR_CIRCLE_code:
	    theButton = O_buttonpressed;
		break;	  
	case SFIR_RIGHT_code:
	    theButton = RIGHT_buttonpressed;
		break;	  
	case SFIR_DOWN_code:
	    theButton = DOWN_buttonpressed;
		break;	  
	case SFIR_REPEAT_code:
	    if (repeatMode==_SFIR_MODE_REPEAT) theButton = theLastButton;
			else theButton = NO_buttonpressed;
		break;	  
	default:
	    theButton = NO_buttonpressed;
	}
	
	// remember last button pressed, in case of repeat
	theLastButton = theButton;
	
  
  resume();
  return(theButton);

}


void IRrecv::toggleRepeatMode() {

  if (repeatMode==_SFIR_MODE_NO_REPEAT) repeatMode = _SFIR_MODE_REPEAT;
    else repeatMode = _SFIR_MODE_NO_REPEAT;


}

// // Decodes the received IR message
// // Returns 0 if no data ready, 1 if data ready.
// // Results of decoding are stored in results
// int IRrecv::decode(decode_results *results) {
  // results->rawbuf = irparams.rawbuf;
  // results->rawlen = irparams.rawlen;
  // if (irparams.rcvstate != STATE_STOP) {
    // return ERR;
  // }
  // if (decodeNEC(results)) {
    // return DECODED;
  // }
  // resume();
  // return ERR;
// }

// decode IR message
long IRrecv::decode(IRdataclass *IRdata) {
	
	// debug
	//Serial.println("decode sez hi");
	//if (irparams.rcvstate != STATE_STOP) return(0);
  long data = 0;
  int offset = 1; // Skip first space
  // Initial mark
  if (!MATCH_MARK(IRdata->rawbuf[offset], NEC_HDR_MARK)) {
    return ERR;
  }
  offset++;
  // Check for repeat
  if (irparams.rawlen == 4 &&
    MATCH_SPACE(IRdata->rawbuf[offset], NEC_RPT_SPACE) &&
    MATCH_MARK(IRdata->rawbuf[offset+1], NEC_BIT_MARK)) {
    IRdata->bits = 0;
    IRdata->value = REPEAT;
    IRdata->code_type = NEC;
    return DECODED;
  }
  if (irparams.rawlen < 2 * NEC_BITS + 4) {
    return ERR;
  }
  // Initial space  
  if (!MATCH_SPACE(IRdata->rawbuf[offset], NEC_HDR_SPACE)) {
    return ERR;
  }
  offset++;
  for (int i = 0; i < NEC_BITS; i++) {
    if (!MATCH_MARK(IRdata->rawbuf[offset], NEC_BIT_MARK)) {
      return ERR;
    }
    offset++;
    if (MATCH_SPACE(IRdata->rawbuf[offset], NEC_ONE_SPACE)) {
      data = (data << 1) | 1;
    } 
    else if (MATCH_SPACE(IRdata->rawbuf[offset], NEC_ZERO_SPACE)) {
      data <<= 1;
    } 
    else {
      return ERR;
    }
    offset++;
  }
  // Success
  IRdata->bits = NEC_BITS;
  IRdata->value    = data;
  IRdata->code_type = NEC;
  return DECODED;
}



