// Trilobot.h
// Trilobot 

#ifndef _TRILOBOT_H
#define _TRILOBOT_H

// kinematic state variables
 typedef struct {
  float ax;
  float ay;
  float w;
  float mx;
  float my;
  float mz;
  float vx;  // x-velocity
  float vy;  // y-velocity
  float Theta;  // heading angle /degrees
} 
KinematicState_t;  // kinematic state 

// machine tick-state variables
// entries hold number of ticks remaining before reversion to default state
typedef struct {
	unsigned LED[6];   // 6 LEDs, turn OFF 
    unsigned motors; // 2 motors, revert to left_motorspeed, right_motorspeed
	}
TickState_t;


// global state variables
//KinematicState_t state;

#include "I2Cdev.h"
#include <avr/pgmspace.h>
#include "Arduino.h"
#include <IRTrilobot.h> // Infrared comms library
#include <IRTrilobotInt.h>
#include <FlexiTimer2.h>

//#include <TRBOT_MPU9150.h>
#define MPU9150_RA_MAG_ADDRESS		0x0C
#define MPU9150_RA_MAG_XOUT_L		0x03
#define MPU9150_RA_MAG_XOUT_H		0x04
#define MPU9150_RA_MAG_YOUT_L		0x05
#define MPU9150_RA_MAG_YOUT_H		0x06
#define MPU9150_RA_MAG_ZOUT_L		0x07
#define MPU9150_RA_MAG_ZOUT_H		0x08

#define TRBOT_MPU9150_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define TRBOT_MPU9150_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define TRBOT_MPU9150_DEFAULT_ADDRESS     TRBOT_MPU9150_ADDRESS_AD0_LOW

#define TRBOT_MPU9150_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define TRBOT_MPU9150_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define TRBOT_MPU9150_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define TRBOT_MPU9150_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define TRBOT_MPU9150_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define TRBOT_MPU9150_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define TRBOT_MPU9150_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define TRBOT_MPU9150_RA_XA_OFFS_L_TC     0x07
#define TRBOT_MPU9150_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define TRBOT_MPU9150_RA_YA_OFFS_L_TC     0x09
#define TRBOT_MPU9150_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define TRBOT_MPU9150_RA_ZA_OFFS_L_TC     0x0B
#define TRBOT_MPU9150_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define TRBOT_MPU9150_RA_XG_OFFS_USRL     0x14
#define TRBOT_MPU9150_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define TRBOT_MPU9150_RA_YG_OFFS_USRL     0x16
#define TRBOT_MPU9150_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define TRBOT_MPU9150_RA_ZG_OFFS_USRL     0x18
#define TRBOT_MPU9150_RA_SMPLRT_DIV       0x19
#define TRBOT_MPU9150_RA_CONFIG           0x1A
#define TRBOT_MPU9150_RA_GYRO_CONFIG      0x1B
#define TRBOT_MPU9150_RA_ACCEL_CONFIG     0x1C
#define TRBOT_MPU9150_RA_FIFO_EN          0x23
#define TRBOT_MPU9150_RA_I2C_MST_CTRL     0x24

#define TRBOT_MPU9150_RA_INT_PIN_CFG      0x37
#define TRBOT_MPU9150_RA_ACCEL_XOUT_H     0x3B
#define TRBOT_MPU9150_RA_ACCEL_XOUT_L     0x3C
#define TRBOT_MPU9150_RA_ACCEL_YOUT_H     0x3D
#define TRBOT_MPU9150_RA_ACCEL_YOUT_L     0x3E
#define TRBOT_MPU9150_RA_ACCEL_ZOUT_H     0x3F
#define TRBOT_MPU9150_RA_ACCEL_ZOUT_L     0x40
#define TRBOT_MPU9150_RA_TEMP_OUT_H       0x41
#define TRBOT_MPU9150_RA_TEMP_OUT_L       0x42
#define TRBOT_MPU9150_RA_GYRO_XOUT_H      0x43
#define TRBOT_MPU9150_RA_GYRO_XOUT_L      0x44
#define TRBOT_MPU9150_RA_GYRO_YOUT_H      0x45
#define TRBOT_MPU9150_RA_GYRO_YOUT_L      0x46
#define TRBOT_MPU9150_RA_GYRO_ZOUT_H      0x47
#define TRBOT_MPU9150_RA_GYRO_ZOUT_L      0x48

#define TRBOT_MPU9150_RA_USER_CTRL        0x6A
#define TRBOT_MPU9150_RA_PWR_MGMT_1       0x6B

#define TRBOT_MPU9150_RA_FIFO_COUNTH      0x72
#define TRBOT_MPU9150_RA_FIFO_COUNTL      0x73
#define TRBOT_MPU9150_RA_FIFO_R_W         0x74
#define TRBOT_MPU9150_RA_WHO_AM_I         0x75

#define TRBOT_MPU9150_TC_OFFSET_BIT       6
#define TRBOT_MPU9150_TC_OFFSET_LENGTH    6


#define TRBOT_MPU9150_CFG_DLPF_CFG_BIT    2
#define TRBOT_MPU9150_CFG_DLPF_CFG_LENGTH 3

#define TRBOT_MPU9150_DLPF_BW_256         0x00
#define TRBOT_MPU9150_DLPF_BW_188         0x01
#define TRBOT_MPU9150_DLPF_BW_98          0x02
#define TRBOT_MPU9150_DLPF_BW_42          0x03
#define TRBOT_MPU9150_DLPF_BW_20          0x04
#define TRBOT_MPU9150_DLPF_BW_10          0x05
#define TRBOT_MPU9150_DLPF_BW_5           0x06

#define TRBOT_MPU9150_GCONFIG_FS_SEL_BIT      4
#define TRBOT_MPU9150_GCONFIG_FS_SEL_LENGTH   2

#define TRBOT_MPU9150_GYRO_FS_250         0x00
#define TRBOT_MPU9150_GYRO_FS_500         0x01
#define TRBOT_MPU9150_GYRO_FS_1000        0x02
#define TRBOT_MPU9150_GYRO_FS_2000        0x03


#define TRBOT_MPU9150_ACONFIG_AFS_SEL_BIT         4
#define TRBOT_MPU9150_ACONFIG_AFS_SEL_LENGTH      2


#define TRBOT_MPU9150_ACCEL_FS_2          0x00
#define TRBOT_MPU9150_ACCEL_FS_4          0x01
#define TRBOT_MPU9150_ACCEL_FS_8          0x02
#define TRBOT_MPU9150_ACCEL_FS_16         0x03

#define TRBOT_MPU9150_I2C_MST_CLK_BIT     3
#define TRBOT_MPU9150_I2C_MST_CLK_LENGTH  4

#define TRBOT_MPU9150_USERCTRL_FIFO_EN_BIT            6
#define TRBOT_MPU9150_USERCTRL_FIFO_RESET_BIT         2

#define TRBOT_MPU9150_PWR1_SLEEP_BIT          6
#define TRBOT_MPU9150_PWR1_CLKSEL_BIT         2
#define TRBOT_MPU9150_PWR1_CLKSEL_LENGTH      3

#define TRBOT_MPU9150_CLOCK_INTERNAL          0x00
#define TRBOT_MPU9150_CLOCK_PLL_XGYRO         0x01

#define TRBOT_MPU9150_WHO_AM_I_BIT        6
#define TRBOT_MPU9150_WHO_AM_I_LENGTH     6


#include <TbotMatrix.h>  // matrix library
#include <EEPROM.h>      // for saving bot-specific calibration data to non-volatile memory

// IR receive modes
#define _SPARKFUN_IR_REMOTE 0


// LED controls
 #define _RIGHT_GREEN_PIN      0
 #define _RIGHT_RED_PIN       1
 #define _FRONT_GREEN_PIN     12
 #define _FRONT_RED_PIN       11
 #define _LEFT_GREEN_PIN       5
 #define _LEFT_RED_PIN        13
 
 
 // #define _RIGHT_GREEN      0
 // #define _RIGHT_RED       1
 // #define _FRONT_GREEN     12
 // #define _FRONT_RED       11
 // #define _LEFT_GREEN       5
 // #define _LEFT_RED        13
 
 // used by LED_colour (sets LEDs to arbitrary colours)
 #define _FRONT_LED_LIGHT 	  0
 #define _LEFT_LED_LIGHT     -1
 #define _RIGHT_LED_LIGHT     1
 
 #define _N_LEDS               6
 #define ON LOW
 #define OFF HIGH
 
 
 // this enum allows LEDs to be addressed by name
 enum LED{
	RIGHT_GREEN,
	RIGHT_RED,
	FRONT_GREEN,
    FRONT_RED,
	LEFT_GREEN,
	LEFT_RED
 };
 
 // modes set by mode buttons on SF IR remote
// response to arrow buttons will be mode-dependent
enum IR_BUTTON_CONTEXT_t {
  Startup_button_context, // power button -> startup mode
  A_button_context,
  B_button_context,
  C_button_context,
  O_button_context,
};

 
 // TRILOBOT_MODE determines what bot does on ticks
 enum TRILOBOT_MODE {
	REMOTE_CONTROL,
	LINE_FOLLOWER,
	LIGHT_FOLLOWER
 };
 
 

// motor control pins
 #define _LEFT_MOTOR_POLARITY_PIN   7
 #define _RIGHT_MOTOR_POLARITY_PIN  8
 #define _LEFT_MOTOR_SPEED_PIN     10
 #define _RIGHT_MOTOR_SPEED_PIN     9
 
 #define _MAX_MOTOR_SPEED 200
 #define _MIN_MOTOR_SPEED 40
 #define _DEFAULT_ACCELERATION 10
 #define _DEFAULT_TURN_DURATION 200
 
 //  LDR pins
 #define _LEFT_LDR_PWR_PIN 		A2
 #define _RIGHT_LDR_PWR_PIN 	A3
 #define _LEFT_LDR_READ_PIN 	A0
 #define _RIGHT_LDR_READ_PIN 	A1
 
 #define _LEFT		0
 #define _RIGHT		1
 
 // Magnetometer 
 #define _N_MAGCAL_DATA			16   // number of calibration points in each direction
 #define _SAMPLE_DT_MAGCAL	    200     // interval between calibration points ms
 #define _MAGCAL_SPEED          200  // turning speed for calibration

 #define _PI                    3.14159
 


const int LEDpin[] = {
			_RIGHT_GREEN_PIN,
			_RIGHT_RED_PIN,
			_FRONT_GREEN_PIN,
			_FRONT_RED_PIN,  
			_LEFT_GREEN_PIN,
			_LEFT_RED_PIN      };
			


 class Trilobot{
 
   // private variables can only be accessed by class methods 
   // these methods should check parameter validity (e.g. indices within bounds)    
   // to provide a safe way for programmers (esp. novices) to drive the hardware
   // without run-time crashes. Such crashes can be hard to trace, because the 
   // effect of an error is mediated by causal links not visible to or accessible 
   // from the Arduino script. 
   private:
   
   
 	 // LED pins
	// LED pins

	// status
	int status_magnetometer_calibration;
   public:
	
  // TRBOT_MPU9150 IMU;	
	 // kinematic observations from MPU9150
	 // x,y acceleration, yaw rate, 3x magnetometer

	 static int16_t ax, ay, w, mx, my, mz; // raw sensor measurements (taken by kineticSense)
   
     static KinematicState_t state; // kinematic state estimates (computed in kineticSense)
	 
	 static TickState_t tickstate; // keeps track of machine state for timed actions
	 
	 // real time clock - ticks updated by interrupt
	 volatile static unsigned long ticks;
	
     // 'listening' introduced to allow IR button presses to be ignored	
	 volatile bool ticked;
  
    TRILOBOT_MODE mode;
    IR_BUTTON_CONTEXT_t buttonContext; 
  
     // parameters
     int Team;
     int Number;
     float* CalibrationData;
     float ReferenceHeading;
	 float Heading;
	 
	 float pi;
	 
	IRrecv IRreceiver; // receiver for IR remote (using NEC codes, e.g. Sparkfun remote COM-11759).
	int IRmode;
	IRdataclass IRdat;

    static int left_motorspeed;
    static int right_motorspeed;
	 
	 // compass
	 float bearing_000;      // compass bearing set direction
	 float bearing;          // angle -180:+180 deg from set direction 
	 float compass_cos;      // cosine component of bearing
	 float compass_sin;      // sine component of bearing
	 
	 //  magnetometer readings are transformed from a circle 
	 // of radius compass_norm centred at (compass_x0, compass_y0) 
	 // to a unit circle at the origin
	 // these to be made private when compass member functions are debugged
	 float compass_x0;        // x-offset for magnetometer readings
	 float compass_y0;        // y-offset
	 float compass_norm;      // radius of circle, centred at (x0,y0),  
	 int compass_x0_EEPROM_address;  // locations in nonvolatile memory
	 int compass_y0_EEPROM_address;  //   where these values are stored
	 int compass_norm_EEPROM_address;//   when the bot is switched off
	 void put_eeprom(int EEPROM_address, const float& value); // nonvolatile storage
	 void get_eeprom(int EEPROM_address, float& value);
    
	 // constructors
    Trilobot();
     
	 // initializer
     void init();
	 
    // timer-interrupt driven tick
	static void tick(void);
	
	 // IR receiver
	 void IRinit(void);
	 int getSparkfunRemoteButtonPress(void);
	 int getIRmessage(void);

	 // LEDS
	 void LED(int whichLED, int state);  // state ON or OFF for LED 0-5
	 // colour 0-255 for 3 bicolour LEDs, _FRONT_LED_LIGHT, _LEFT_LED_LIGHT, _RIGHT_LED_LIGHT
	 void LED_colour(int whichLED, int red, int green);  
	 void LEDoff(void);  // all LEDs off
	 
	 void tickFlash(int WhichLED, unsigned Nticks);  // turn on LED for number of ticks
	 
     // light sensors (LDRs)
	 // returns a number between 0 and 1024
	 int readLightsensor(int side);
	 
	 // kinematic state measurements
	 static void kineticSense(void); // obtain accel, gyro & magnet measurements from MPU9150

	 static void stateUpdate(void);
	 
     float compass(void);	 
	 
	 
	 
     void forward(int  speed); // both motors at this speed
	 void turn(int direction, int msecs);  // turn by shutting 1 motor
	 void turn(int direction) {turn(direction, _DEFAULT_TURN_DURATION);}  
	 void accelerate(int step);
	 void proceed(void);
	 void motors(int LeftMotorSpeed, int RightMotorSpeed);
	 void tickTurn(int direction, int Nticks);
	 
	 // waggle dance
	 void waggle(int stepLen, int numSteps); 
	 
	 
//     void accelerate(int  dSpeed);
//     void setTheta(int newTheta);
//     void turnby(int dTheta);
//     void  setSpin(int  Spin); 

        bool testConnection();


        void setRate(uint8_t rate);
        
        uint8_t  checkMagStatus();

        void setDLPFMode(uint8_t bandwidth);

        void setFullScaleGyroRange(uint8_t range);

        void setFullScaleAccelRange(uint8_t range);
  
        void setMasterClockSpeed(uint8_t speed);


        // int16_t X();
        // int16_t Y();

        // // temperature
        // int16_t T();

        // // rotation 
        // int16_t R();
		
		// update kinematic observations (private members)
		// x,y acceleration, w = yaw rate, & 3x raw magnetometer readings
		void update(KinematicState_t state);

        // magnetometer
        void updateMag(int16_t* mx, int16_t* my, int16_t* mz );

        void setFIFOEnabled(bool enabled);
         void resetFIFO();
       void setSleepEnabled(bool enabled);
       void setClockSource(uint8_t source);

   
        // FIFO_COUNT_* registers
        uint16_t getFIFOCount();

        // FIFO_R_W register
        uint8_t getFIFOByte();
   //     void setFIFOByte(uint8_t data);
        void getFIFOBytes(uint8_t *data, uint8_t length);

        // WHO_AM_I register
        uint8_t getDeviceID();
       
        // ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========
        
        int8_t getXGyroOffset();
        void setXGyroOffset(int8_t offset);

        // YG_OFFS_TC register
        int8_t getYGyroOffset();
        void setYGyroOffset(int8_t offset);

        // ZG_OFFS_TC register
        int8_t getZGyroOffset();
        void setZGyroOffset(int8_t offset);

        // X_FINE_GAIN register
        int8_t getXFineGain();
        void setXFineGain(int8_t gain);

        // Y_FINE_GAIN register
        int8_t getYFineGain();
        void setYFineGain(int8_t gain);

        // Z_FINE_GAIN register
        int8_t getZFineGain();
        void setZFineGain(int8_t gain);

        // XA_OFFS_* registers
        int16_t getXAccelOffset();
        void setXAccelOffset(int16_t offset);

        // YA_OFFS_* register
        int16_t getYAccelOffset();
        void setYAccelOffset(int16_t offset);

        // ZA_OFFS_* register
        int16_t getZAccelOffset();
        void setZAccelOffset(int16_t offset);

        // XG_OFFS_USR* registers
        int16_t getXGyroOffsetUser();
        void setXGyroOffsetUser(int16_t offset);

        // YG_OFFS_USR* register
        int16_t getYGyroOffsetUser();
        void setYGyroOffsetUser(int16_t offset);

        // ZG_OFFS_USR* register
        int16_t getZGyroOffsetUser();
        void setZGyroOffsetUser(int16_t offset);
        
    private:
        static uint8_t devAddr;
        static uint8_t buffer[6];
    
 };
 



 
#endif //_TRILOBOT_H
