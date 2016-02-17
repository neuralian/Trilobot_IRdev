// Trilobot.cpp
 // Trilobot members and methods
 // Mike Paulin 2015
 
#include "Trilobot.h"
//#include "MGP9150.h"

uint8_t Trilobot::devAddr = TRBOT_MPU9150_DEFAULT_ADDRESS;

// raw motion sensor data
uint8_t Trilobot::buffer[6] = {0,0,0,0,0,0,};
int16_t Trilobot::ax=0.;
int16_t Trilobot::ay=0.;
int16_t Trilobot::w=0.;
int16_t Trilobot::mx=0.;
int16_t Trilobot::my=0.;
int16_t Trilobot::mz=0.;
volatile unsigned long Trilobot::ticks=0;  // tick counter

// state estimates
KinematicState_t Trilobot::state = {0.,0.,0.,0.,0.,0.,0.,0.,0.};

TickState_t Trilobot::tickstate;
int Trilobot::left_motorspeed=0;
int Trilobot::right_motorspeed=0;

 
 int _MOTOR_POLARITY[] = {_LEFT_MOTOR_POLARITY_PIN, _RIGHT_MOTOR_POLARITY_PIN};
 int _MOTOR_SPEED[]    = {_LEFT_MOTOR_SPEED_PIN, _RIGHT_MOTOR_SPEED_PIN};
 
 // constructors
 Trilobot::Trilobot(){
  Team = 0;
  Number = 0;
  CalibrationData = 0;
  ReferenceHeading = 0; 
  devAddr = TRBOT_MPU9150_DEFAULT_ADDRESS;
 }
 
 


void Trilobot::init(){

  int i;
  
  for(i=0; i<6; i++) tickstate.LED[i]=0;
  tickstate.motors=0;
  
   
  
  // LED pins as OUTPUTs, LEDs off
  for (i =0; i<_N_LEDS; i++) {
     pinMode(LEDpin[i], OUTPUT);
	 digitalWrite(LEDpin[i], OFF);
	 }
	 
	// if(Team==1)  onLED(2); 
  
  // motor polarity (spin direction) pins as outputs
  pinMode(_LEFT_MOTOR_POLARITY_PIN, OUTPUT);
  pinMode(_RIGHT_MOTOR_POLARITY_PIN, OUTPUT);
  
  // LDR init
  pinMode(_LEFT_LDR_PWR_PIN, OUTPUT);
  pinMode(_RIGHT_LDR_PWR_PIN, OUTPUT);
  
  // initial speed 0
   analogWrite(_LEFT_MOTOR_SPEED_PIN, 0);
   analogWrite(_RIGHT_MOTOR_SPEED_PIN, 0); 
   
   // compass parameters
   	ReferenceHeading = 0; 
	pi = _PI;
	get_eeprom(compass_x0_EEPROM_address, compass_x0);
	get_eeprom(compass_y0_EEPROM_address, compass_y0);
	get_eeprom(compass_norm_EEPROM_address, compass_norm);
	
	left_motorspeed = _MIN_MOTOR_SPEED;
	right_motorspeed = _MIN_MOTOR_SPEED;
	

	
	// EEPROM addresses for compass calibrations
	compass_x0_EEPROM_address = 0;  
	compass_y0_EEPROM_address = compass_x0_EEPROM_address + sizeof(float);  
	compass_norm_EEPROM_address = compass_y0_EEPROM_address + sizeof(float);
	
	// read stored magnetometer calibrations
	 get_eeprom(compass_x0_EEPROM_address, compass_x0);        // x-offset 
	 get_eeprom(compass_y0_EEPROM_address, compass_y0);        // y-offset 
	 get_eeprom(compass_norm_EEPROM_address, compass_norm);    // radius for magnetometer readings

   Trilobot::IRinit(); // initialize receiver for Sparkfun remote
  

	 
	FlexiTimer2::set(50, tick); // timer interrupt interval & function to call
    FlexiTimer2::start();
	 
	 // IMU
	setClockSource(TRBOT_MPU9150_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(TRBOT_MPU9150_GYRO_FS_250);
    setFullScaleAccelRange(TRBOT_MPU9150_ACCEL_FS_2);
    setSleepEnabled(false); 
	
	// stopped
	forward(0);
	ticked = true;
	 
}


void Trilobot::tick(void)
{ 
   int i;
   
  // increment tick count
  ticks++;
  
  // read 9df motion sensor
  // nb need to enable interrupts for I2C clock
  sei();
  kineticSense();
  cli();
  
  
  // decrement TickState counters
  for (i=0; i<6; i++) {
	if (tickstate.LED[i]>0)  tickstate.LED[i]--;
	if (tickstate.LED[i]==0) digitalWrite(LEDpin[i], OFF);
  }
  if (tickstate.motors>0) tickstate.motors--;
  if (tickstate.motors==0) {
	analogWrite(_LEFT_MOTOR_SPEED_PIN, left_motorspeed);
	analogWrite(_RIGHT_MOTOR_SPEED_PIN, right_motorspeed);
  }
}

void Trilobot::IRinit(void){

	     IRreceiver.init();

   }
   
   

int Trilobot::getSparkfunRemoteButtonPress(){


	return(IRreceiver.getSparkfunRemoteButtonPress());

}



int Trilobot::getIRmessage(){
	
	return(IRreceiver.getIRmessage(&IRdat));
}

// LED switch
// nb LEDs turn on if pulled LOW.  ON and OFF are defined in Trilobot.h
// so that state can be specified as either LOW/ON or HIGH/OFF
void Trilobot::LED(int i, int state) { 
   if (i>=0 && i<6) digitalWrite(LEDpin[i], state);
}

// set LED colours, 
// 0-255 for 3 bicolour LEDs, _FRONT_LED_LIGHT, _LEFT_LED_LIGHT, _RIGHT_LED_LIGHT
void Trilobot::LED_colour(int whichLED, int red, int green){

   switch (whichLED) {
      case _FRONT_LED_LIGHT:
	    analogWrite(_FRONT_RED_PIN, 255-red);
	    analogWrite(_FRONT_GREEN_PIN, 255-green);
		break;
      case _LEFT_LED_LIGHT:
	    analogWrite(_LEFT_RED_PIN, 255-red);
	    analogWrite(_LEFT_GREEN_PIN, 255-green);
		break;      
	  case _RIGHT_LED_LIGHT:
	    analogWrite(_RIGHT_RED_PIN, 255-red);
	    analogWrite(_RIGHT_GREEN_PIN, 255-green);
		break;		
		}
}

// all LEDs off
void Trilobot::LEDoff(void){
	for (int i=0; i<6; i++) digitalWrite(LEDpin[i], OFF);
}

void Trilobot::tickFlash(int WhichLED, unsigned Nticks){
	
	digitalWrite(LEDpin[WhichLED], ON);
	tickstate.LED[WhichLED] = Nticks;

}



// LDRs 
int Trilobot::readLightsensor(int side) { 
    int value = -1;
    if (side==_LEFT) {
		digitalWrite(_LEFT_LDR_PWR_PIN, HIGH);
		value = analogRead(_LEFT_LDR_READ_PIN);
		digitalWrite(_LEFT_LDR_PWR_PIN, LOW);
		}
	else {
		digitalWrite(_RIGHT_LDR_PWR_PIN, HIGH);
		value = analogRead(_RIGHT_LDR_READ_PIN);
		digitalWrite(_RIGHT_LDR_PWR_PIN, LOW);
		}
    return(value);
}


// // magnetometer. update mx,my,mz magnetic field properties
// void Trilobot::magnetSense(void){ 
     // IMU.updateMag(&_mx,&_my, &_mz);
// }

// read and write to EEPROM
void Trilobot::put_eeprom(int EEPROM_address, const float& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(EEPROM_address++, *p++);
}

void Trilobot::get_eeprom(int EEPROM_address, float& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(EEPROM_address++);
}



void Trilobot::forward(int speed){
   left_motorspeed  = speed;
   right_motorspeed = speed;   
   analogWrite(_LEFT_MOTOR_SPEED_PIN, speed);
   analogWrite(_RIGHT_MOTOR_SPEED_PIN, speed);
}

// accelerate/decelerate 
void Trilobot::accelerate(int increment) {
   // increment up to max speed or down to zero
   left_motorspeed  = max(min(left_motorspeed  + increment, 
						_MAX_MOTOR_SPEED),_MIN_MOTOR_SPEED);
   right_motorspeed = max(min(right_motorspeed + increment, 
						_MAX_MOTOR_SPEED),_MIN_MOTOR_SPEED);
  Serial.print(left_motorspeed); Serial.print(", "); Serial.println(right_motorspeed);
   analogWrite(_LEFT_MOTOR_SPEED_PIN, left_motorspeed>_MIN_MOTOR_SPEED? left_motorspeed:0);
   analogWrite(_RIGHT_MOTOR_SPEED_PIN, right_motorspeed>_MIN_MOTOR_SPEED? right_motorspeed:0);
   
}

void Trilobot::turn(int direction, int msecs){
    // shut down the motor on the opposite side
	if (direction==_LEFT) analogWrite(_LEFT_MOTOR_SPEED_PIN, 0);
      else analogWrite(_RIGHT_MOTOR_SPEED_PIN, 0);
}

void Trilobot::tickTurn(int direction, int Nticks){
    // shut down the motor on the opposite side
	if (direction==_LEFT) analogWrite(_LEFT_MOTOR_SPEED_PIN, 0);
      else analogWrite(_RIGHT_MOTOR_SPEED_PIN, 0);
    tickstate.motors = Nticks;	  
}


void Trilobot::proceed(){
	analogWrite(_LEFT_MOTOR_SPEED_PIN, left_motorspeed);
	analogWrite(_RIGHT_MOTOR_SPEED_PIN, right_motorspeed);	
}

void Trilobot::motors(int LeftMotorSpeed, int RightMotorSpeed){
   left_motorspeed  = LeftMotorSpeed;
   right_motorspeed = RightMotorSpeed;   
   analogWrite(_LEFT_MOTOR_SPEED_PIN, left_motorspeed);
   analogWrite(_RIGHT_MOTOR_SPEED_PIN, right_motorspeed);
}

void Trilobot::waggle(int stepAngle, int numSteps){
	forward(_MAX_MOTOR_SPEED);
    for (int i=0; i<numSteps; i++) {
		while (compass()>-5) tickTurn(_LEFT,2);
		while (compass()<5) tickTurn(_RIGHT,2);	
		// analogWrite(_LEFT_MOTOR_SPEED_PIN, _MAX_MOTOR_SPEED);
		// analogWrite(_RIGHT_MOTOR_SPEED_PIN, 0);
        // delay(500);
		// analogWrite(_LEFT_MOTOR_SPEED_PIN, 0 );
		// analogWrite(_RIGHT_MOTOR_SPEED_PIN, _MAX_MOTOR_SPEED);	
		// delay(500);
	}
		
}	 

// compass heading in radians
float Trilobot::compass(void) {
  
  //  bot.magnetSense();
    float x = (float(state.mx) - compass_x0)/compass_norm;              
//    float x = (float(bot.mx()) - bot.compass_x0)/bot.compass_norm;              
    float y =(float(state.my) - compass_y0)/compass_norm;
    
    
    float theta = atan2(x,y);// radians, -pi:+pi from 0
    
//  theta = (theta>0)? theta:(2.*bot.pi+theta); // -> clockwise from 0
  theta = 180.*theta/pi;
    
    return(theta);   
}


//     void setSpeed(int  newSpeed);
//     void accelerate(int  dSpeed);
//     void setTheta(int newTheta);
//     void turnby(int dTheta);
//     void  setSpin(int  Spin);


// // constructors
// Trilobot::Trilobot() {
    // devAddr = Trilobot_DEFAULT_ADDRESS;
// }

// Trilobot::Trilobot(uint8_t address) {
    // devAddr = address;
// }

// init
// void Trilobot::initialize() {
    // setClockSource(Trilobot_CLOCK_PLL_XGYRO);
    // setFullScaleGyroRange(Trilobot_GYRO_FS_250);
    // setFullScaleAccelRange(Trilobot_ACCEL_FS_2);
    // setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
// }

// Verify the I2C connection.
bool Trilobot::testConnection() {
    return getDeviceID() == 0x34;
}

// set gyro sample rate divider
void Trilobot::setRate(uint8_t rate) {
    I2Cdev::writeByte(devAddr, TRBOT_MPU9150_RA_SMPLRT_DIV, rate);
}

/** Digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see Trilobot_RA_CONFIG
 * @see Trilobot_CFG_DLPF_CFG_BIT
 * @see Trilobot_CFG_DLPF_CFG_LENGTH
 */
// uint8_t Trilobot::getDLPFMode() {
    // I2Cdev::readBits(devAddr, Trilobot_RA_CONFIG, Trilobot_CFG_DLPF_CFG_BIT, Trilobot_CFG_DLPF_CFG_LENGTH, buffer);
    // return buffer[0];
// }
// Set digital low-pass filter configuration.
void Trilobot::setDLPFMode(uint8_t mode) {
    I2Cdev::writeBits(devAddr, TRBOT_MPU9150_RA_CONFIG, TRBOT_MPU9150_CFG_DLPF_CFG_BIT, TRBOT_MPU9150_CFG_DLPF_CFG_LENGTH, mode);
}


/** gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 */

// Set full-scale gyroscope range.
void Trilobot::setFullScaleGyroRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, TRBOT_MPU9150_RA_GYRO_CONFIG, TRBOT_MPU9150_GCONFIG_FS_SEL_BIT, TRBOT_MPU9150_GCONFIG_FS_SEL_LENGTH, range);
}

/**  accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see TRBOT_MPU9150_ACCEL_FS_2
 * @see TRBOT_MPU9150_RA_ACCEL_CONFIG
 * @see TRBOT_MPU9150_ACONFIG_AFS_SEL_BIT
 * @see TRBOT_MPU9150_ACONFIG_AFS_SEL_LENGTH
 */

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void Trilobot::setFullScaleAccelRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, TRBOT_MPU9150_RA_ACCEL_CONFIG, TRBOT_MPU9150_ACONFIG_AFS_SEL_BIT, TRBOT_MPU9150_ACONFIG_AFS_SEL_LENGTH, range);
}

/**  I2C master clock speed.
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 *
 * @return Current I2C master clock speed
 * @see TRBOT_MPU9150_RA_I2C_MST_CTRL
 */

// Set I2C master clock speed.
void Trilobot::setMasterClockSpeed(uint8_t speed) {
    I2Cdev::writeBits(devAddr, TRBOT_MPU9150_RA_I2C_MST_CTRL, TRBOT_MPU9150_I2C_MST_CLK_BIT, TRBOT_MPU9150_I2C_MST_CLK_LENGTH, speed);
}

// update accelerometer, gyro and magnetometer observations
void Trilobot::kineticSense(void) {
						
	// accelerometer (x & y)
	I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_ACCEL_XOUT_H, 2, buffer);
    ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_ACCEL_YOUT_H, 2, buffer);
    ay = (((int16_t)buffer[0]) << 8) | buffer[1];
	
	// gyro (yaw rate)
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_GYRO_ZOUT_H, 2, buffer);
    w = (((int16_t)buffer[0]) << 8) | buffer[1];
    
	// magnetometer
	I2Cdev::writeByte(devAddr, TRBOT_MPU9150_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
	delay(10);
	I2Cdev::writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
	delay(10);
	I2Cdev::readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer);
	mx = (((int16_t)buffer[1]) << 8) | buffer[0];
    my = (((int16_t)buffer[3]) << 8) | buffer[2];
    mz = (((int16_t)buffer[5]) << 8) | buffer[4];

    stateUpdate();
	
}

void Trilobot::stateUpdate(void){
	
	
	
	// dummy state estimator - copies raw measurements to state vector
	state.ax = ax;
	state.ay = ay;
	state.w = w;
	state.mx = 0.95*state.mx + 0.05*mx;
	state.my = 0.95*state.my + 0.05*my;
	state.mz = 0.95*state.mz + 0.05*mz;
	
	
	
};

void Trilobot::updateMag(int16_t* mx, int16_t* my, int16_t* mz ) {
    
	//read mag
	I2Cdev::writeByte(devAddr, TRBOT_MPU9150_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
	delay(10);
	I2Cdev::writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
	delay(10);
	I2Cdev::readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer);
	*mx = (((int16_t)buffer[1]) << 8) | buffer[0];
    *my = (((int16_t)buffer[3]) << 8) | buffer[2];
    *mz = (((int16_t)buffer[5]) << 8) | buffer[4];		
}


// //  X-axis accelerometer
// int16_t Trilobot::X() {
    // I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_ACCEL_XOUT_H, 2, buffer);
    // return (((int16_t)buffer[0]) << 8) | buffer[1];
// }
// // Y-axis accelerometer 
// int16_t Trilobot::Y() {
    // I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_ACCEL_YOUT_H, 2, buffer);
    // return (((int16_t)buffer[0]) << 8) | buffer[1];
// }

// // temperature.
// int16_t Trilobot::T() {
    // I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_TEMP_OUT_H, 2, buffer);
    // return (((int16_t)buffer[0]) << 8) | buffer[1];
// }

// // Z-axis gyro.
// int16_t Trilobot::R() {
    // I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_GYRO_ZOUT_H, 2, buffer);
    // return (((int16_t)buffer[0]) << 8) | buffer[1];
// }

// Set FIFO enabled status.
void Trilobot::setFIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, TRBOT_MPU9150_RA_USER_CTRL, TRBOT_MPU9150_USERCTRL_FIFO_EN_BIT, enabled);
}

// Reset  FIFO.
void Trilobot::resetFIFO() {
    I2Cdev::writeBit(devAddr, TRBOT_MPU9150_RA_USER_CTRL, TRBOT_MPU9150_USERCTRL_FIFO_RESET_BIT, true);
}

// sleep mode status.
void Trilobot::setSleepEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, TRBOT_MPU9150_RA_PWR_MGMT_1, TRBOT_MPU9150_PWR1_SLEEP_BIT, enabled);
}


/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t Trilobot::getFIFOCount() {
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
uint8_t Trilobot::getFIFOByte() {
    I2Cdev::readByte(devAddr, TRBOT_MPU9150_RA_FIFO_R_W, buffer);
    return buffer[0];
}
void Trilobot::getFIFOBytes(uint8_t *data, uint8_t length) {
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_FIFO_R_W, length, data);
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34)
 * @see TRBOT_MPU9150_RA_WHO_AM_I
 * @see TRBOT_MPU9150_WHO_AM_I_BIT
 * @see TRBOT_MPU9150_WHO_AM_I_LENGTH
 */
uint8_t Trilobot::getDeviceID() {
    I2Cdev::readBits(devAddr, TRBOT_MPU9150_RA_WHO_AM_I, TRBOT_MPU9150_WHO_AM_I_BIT, TRBOT_MPU9150_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}


int8_t Trilobot::getXGyroOffset() {
    I2Cdev::readBits(devAddr, TRBOT_MPU9150_RA_XG_OFFS_TC, TRBOT_MPU9150_TC_OFFSET_BIT, TRBOT_MPU9150_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void Trilobot::setXGyroOffset(int8_t offset) {
    I2Cdev::writeBits(devAddr, TRBOT_MPU9150_RA_XG_OFFS_TC, TRBOT_MPU9150_TC_OFFSET_BIT, TRBOT_MPU9150_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register

int8_t Trilobot::getYGyroOffset() {
    I2Cdev::readBits(devAddr, TRBOT_MPU9150_RA_YG_OFFS_TC, TRBOT_MPU9150_TC_OFFSET_BIT, TRBOT_MPU9150_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void Trilobot::setYGyroOffset(int8_t offset) {
    I2Cdev::writeBits(devAddr, TRBOT_MPU9150_RA_YG_OFFS_TC, TRBOT_MPU9150_TC_OFFSET_BIT, TRBOT_MPU9150_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register

int8_t Trilobot::getZGyroOffset() {
    I2Cdev::readBits(devAddr, TRBOT_MPU9150_RA_ZG_OFFS_TC, TRBOT_MPU9150_TC_OFFSET_BIT, TRBOT_MPU9150_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void Trilobot::setZGyroOffset(int8_t offset) {
    I2Cdev::writeBits(devAddr, TRBOT_MPU9150_RA_ZG_OFFS_TC, TRBOT_MPU9150_TC_OFFSET_BIT, TRBOT_MPU9150_TC_OFFSET_LENGTH, offset);
}

// X_FINE_GAIN register

int8_t Trilobot::getXFineGain() {
    I2Cdev::readByte(devAddr, TRBOT_MPU9150_RA_X_FINE_GAIN, buffer);
    return buffer[0];
}
void Trilobot::setXFineGain(int8_t gain) {
    I2Cdev::writeByte(devAddr, TRBOT_MPU9150_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register

int8_t Trilobot::getYFineGain() {
    I2Cdev::readByte(devAddr, TRBOT_MPU9150_RA_Y_FINE_GAIN, buffer);
    return buffer[0];
}
void Trilobot::setYFineGain(int8_t gain) {
    I2Cdev::writeByte(devAddr, TRBOT_MPU9150_RA_Y_FINE_GAIN, gain);
}

// Z_FINE_GAIN register

int8_t Trilobot::getZFineGain() {
    I2Cdev::readByte(devAddr, TRBOT_MPU9150_RA_Z_FINE_GAIN, buffer);
    return buffer[0];
}
void Trilobot::setZFineGain(int8_t gain) {
    I2Cdev::writeByte(devAddr, TRBOT_MPU9150_RA_Z_FINE_GAIN, gain);
}

// XA_OFFS_* registers

int16_t Trilobot::getXAccelOffset() {
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_XA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void Trilobot::setXAccelOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, TRBOT_MPU9150_RA_XA_OFFS_H, offset);
}

// YA_OFFS_* register

int16_t Trilobot::getYAccelOffset() {
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_YA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void Trilobot::setYAccelOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, TRBOT_MPU9150_RA_YA_OFFS_H, offset);
}

// ZA_OFFS_* register

int16_t Trilobot::getZAccelOffset() {
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_ZA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void Trilobot::setZAccelOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, TRBOT_MPU9150_RA_ZA_OFFS_H, offset);
}

// XG_OFFS_USR* registers

int16_t Trilobot::getXGyroOffsetUser() {
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void Trilobot::setXGyroOffsetUser(int16_t offset) {
    I2Cdev::writeWord(devAddr, TRBOT_MPU9150_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register

int16_t Trilobot::getYGyroOffsetUser() {
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void Trilobot::setYGyroOffsetUser(int16_t offset) {
    I2Cdev::writeWord(devAddr, TRBOT_MPU9150_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register

int16_t Trilobot::getZGyroOffsetUser() {
    I2Cdev::readBytes(devAddr, TRBOT_MPU9150_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void Trilobot::setZGyroOffsetUser(int16_t offset) {
    I2Cdev::writeWord(devAddr, TRBOT_MPU9150_RA_ZG_OFFS_USRH, offset);
}



/** clock source 
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see TRBOT_MPU9150_RA_PWR_MGMT_1
 * @see TRBOT_MPU9150_PWR1_CLKSEL_BIT
 * @see TRBOT_MPU9150_PWR1_CLKSEL_LENGTH
 // */
void Trilobot::setClockSource(uint8_t source) {
    I2Cdev::writeBits(devAddr, TRBOT_MPU9150_RA_PWR_MGMT_1, TRBOT_MPU9150_PWR1_CLKSEL_BIT, TRBOT_MPU9150_PWR1_CLKSEL_LENGTH, source);
}

// COPYRIGHT NOTICE
// I2Cdev library collection - Trilobot I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

