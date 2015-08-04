#include <Arduino.h>
#include <SPI.h>
#include <RF24_config.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <PID_v1.h>
#include <Servo.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include <Wire.h>
#endif

//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

#include "MPU6050_6Axis_MotionApps20.h"

#include "Propeller.h"


#define PI 3.141592
#define RAD2DEG( Rads ) Rads * ( 180.0 / PI )
#define DEG2RAD( Degs ) ( PI / 180.0 ) * Degs

#define Pin_FL A0
#define Pin_FR A1
#define Pin_RL A2
#define Pin_RR A3

#define CE_PIN  9
#define CSN_PIN 10

double Pitch, Yaw, Roll;

double Throttle_Pitch;
double Throttle_Yaw;
double Throttle_Roll;

double SetPoint_Pitch = 0.0;
double SetPoint_Yaw = 0.0;
double SetPoint_Roll = 0.0;

#define PITCH_P 0.225
#define PITCH_I 0.0
#define PITCH_D 0.1

#define ROLL_P 0.2
#define ROLL_I 0.0
#define ROLL_D 0.06

#define YAW_P 0.25
#define YAW_I 0
#define YAW_D 0

#define PITCH_MAX DEG2RAD( 10.0 )
#define PITCH_MIN DEG2RAD( -10.0 )

#define ROLL_MAX DEG2RAD( 10.0 )
#define ROLL_MIN DEG2RAD( -10.0 )


#define YAW_MAX DEG2RAD( 180.0 )
#define YAW_MIN DEG2RAD( -180.0 )
#define YAW_INFLUENCE 0

PID PID_Pitch( &Pitch, &Throttle_Pitch, &SetPoint_Pitch, PITCH_P, PITCH_I, PITCH_D, REVERSE );
PID PID_Roll( &Roll, &Throttle_Roll, &SetPoint_Roll, ROLL_P, ROLL_I, ROLL_D, REVERSE );
PID PID_Yaw( &Yaw, &Throttle_Yaw, &SetPoint_Yaw, YAW_P, YAW_I, YAW_D, REVERSE );

#define PIDCount 3
PID PIDs[ PIDCount ] = { PID_Pitch, PID_Roll, PID_Yaw };

RF24 Radio( CE_PIN, CSN_PIN );
#define RADIO_PIPE 0xABCDABCD71LL

// [ 1, 2 ] [ 3, 4 ] = Left stick, right stick.
byte Joystick[ 4 ];
float Joystick_Float[ 4 ];

#define CORRECTION_MAX 1.0
#define ESC_MAX 1.0
#define ESC_ARM_DELAY 3000
#define THROTTLE_MIN 0.1

#define DEBUG
#define DEBUG_ANGLES
//#define DEBUG_THROTTLE
//#define DEBUG_JOYSTICKS
//#define ESC_CALIBRATION

long LastRadioLoss;
#define RadioLossPeriod 500

float tFL, tFR, tRL, tRR;

Propeller FL, FR, RL, RR;
Propeller Props_CW[ ] = { FR, RL };
Propeller Props_CCW[ ] = { FL, RR };

MPU6050 mpu;
// MPU control/status vars
bool dmpReady;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[ 64 ]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[ 3 ];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady( )
{
  mpuInterrupt = true;
}

void setup( )
{
	
#ifdef DEBUG
	Serial.begin( 115200 );
	delay( 2000 );
	Serial.println( "Props" );
#endif

	init_Propellers( );

#ifdef DEBUG
	Serial.println( "Radio" );
#endif

	init_Radio( );

#ifdef DEBUG
	Serial.println( "PID" );
#endif

	init_PIDs( );

#ifdef DEBUG
	Serial.println( "Gyro" );
#endif

	init_MPU( );

#ifdef DEBUG
	Serial.println( "Done" );
#endif

}

void loop( )
{
	HandleRadio( );
	SetGyroValues( );
	UpdatePIDs( );
}

void UpdateThrottles( void )
{
	float Throttle = Joystick_Float[ 3 ];
	float P = Throttle_Pitch;
#ifdef DEBUG_THROTTLE
	Serial.println( Throttle_Pitch );
	Serial.println( Throttle_Yaw );
	Serial.println( Throttle_Roll );
#endif
	float R = Throttle_Roll;
	float Y = Throttle_Yaw;

	float vCW = ( 1.0 - Y * YAW_INFLUENCE ) * Throttle;
	float vCCW = ( 1.0 + Y * YAW_INFLUENCE ) * Throttle;

#ifdef DEBUG_THROTTLE
	Serial.print( "CW:\t" );
	Serial.println( vCW );
	Serial.print( "CCW:\t" );
	Serial.println( vCCW );
#endif

	tFL = ( 1.0 + ( P + R ) ) * vCW;
	tRR = ( 1.0 + ( -P - R ) ) * vCW;

	tFR = ( 1.0 + ( P - R ) ) * vCCW;
	tRL = ( 1.0 + ( -P + R ) ) * vCCW;

	FL.SetThrottle( byte( constrain( tFL, 0.0, 1.0 ) * 255 ) );
	FR.SetThrottle( byte( constrain( tFR, 0.0, 1.0 ) * 255 ) );
	RL.SetThrottle( byte( constrain( tRL, 0.0, 1.0 ) * 255 ) );
	RR.SetThrottle( byte( constrain( tRR, 0.0, 1.0 ) * 255 ) );
#ifdef DEBUG_THROTTLE
	Serial.print( "FL: " );
	Serial.println( byte( constrain( tFL, 0.0, 1.0 ) * 255 ) );
	Serial.print( "FR: " );
	Serial.println( byte( constrain( tFR, 0.0, 1.0 ) * 255 ) );
	Serial.print( "RL: " );
	Serial.println( byte( constrain( tRL, 0.0, 1.0 ) * 255 ) );
	Serial.print( "RR: " );
	Serial.println( byte( constrain( tRR, 0.0, 1.0 ) * 255 ) );
	Serial.println( );
#endif
}

void UpdatePIDs( )
{
	if ( Joystick_Float[ 3 ] <= THROTTLE_MIN )
	{
		FL.SetThrottle( 0 );
		FR.SetThrottle( 0 );
		RL.SetThrottle( 0 );
		RR.SetThrottle( 0 );
		return;
	}
	bool Change = false;
	for ( int Q = 0; Q < PIDCount; Q++ )
		Change = PIDs[ Q ].Compute( ) || Change;

	if ( Change )
		UpdateThrottles( );
}

void HandleRadio( void )
{
	bool Available = Radio.available( );
	if ( !Available && millis( ) - LastRadioLoss >= RadioLossPeriod )
	{
		for ( int Q = 0; Q < 4; Q++ )
			Joystick_Float[ Q ] = 0.0;

#ifdef DEBUG_JOYSTICKS
		Serial.println( "No radio available." );
#endif
		return;
	}

	while ( Available )
	{
		Radio.read( Joystick, sizeof( Joystick ) );
		for ( int Q = 0; Q < 4; Q++ )
			Joystick_Float[ Q ] = float( Joystick[ Q ] / 255.0 );
	
		Available = Radio.available( );
	}
	//byte Throttles[ ] = { byte( tFL * 255 ), byte( tFR * 255 ), byte( tRL * 255 ), byte( tRR * 255 ) };
	//Radio.writeAckPayload( 1, &Throttles, sizeof( Throttles ) );
	LastRadioLoss = millis( );
	// Fetch the data payload
	

#ifdef DEBUG_JOYSTICKS
	Serial.print( "X = " );
	Serial.print( Joystick_Float[ 0 ] );
	Serial.print( " Y = " );
	Serial.println( Joystick_Float[ 1 ] );

	Serial.print( "X2 = " );
	Serial.print( Joystick_Float[ 2 ] );
	Serial.print( " Y2 = " );
	Serial.println( Joystick_Float[ 3 ] );
#endif
}

void SetGyroValues( void )
{
	// if programming failed, don't try to do anything
  if ( !dmpReady ) return;

  // wait for MPU interrupt or extra packet(s) available
  while ( !mpuInterrupt && fifoCount < packetSize )
  {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus( );

  // get current FIFO count
  fifoCount = mpu.getFIFOCount( );
  // check for overflow (this should never happen unless our code is too inefficient)
  if ( ( mpuIntStatus & 0x10 ) || fifoCount == 1024 )
  {
    // reset so we can continue cleanly
    mpu.resetFIFO( );
#ifdef DEBUG
    Serial.println( F( "FIFO overflow!" ) );
#endif

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if ( mpuIntStatus & 0x02 )
  {
    // wait for correct available data length, should be a VERY short wait
    while ( fifoCount < packetSize ) fifoCount = mpu.getFIFOCount( );

    // read a packet from FIFO
    mpu.getFIFOBytes( fifoBuffer, packetSize );

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion( &q, fifoBuffer );
    mpu.dmpGetGravity( &gravity, &q );
    mpu.dmpGetYawPitchRoll( ypr, &q, &gravity );
  }

	Yaw = double( ypr[ 0 ] );
	Pitch = double( ypr[ 1 ] );
	Roll = double( ypr[ 2 ] );

#ifdef DEBUG_ANGLES
	Serial.print( "Pitch: Rad: " );
	Serial.print( Pitch );
	Serial.print( ", Deg: " );
	Serial.println( RAD2DEG( Pitch ) );

	Serial.print( "Yaw: Rad: " );
	Serial.print( Yaw );
	Serial.print( ", Deg: " );
	Serial.println( RAD2DEG( Yaw ) );
	
	Serial.print( "Roll: Rad: " );
	Serial.print( Roll );
	Serial.print( ", Deg: " );
	Serial.println( RAD2DEG( Roll ) );
	Serial.println( );
#endif
}

void init_MPU( void )
{
  dmpReady = false;
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin( );
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup( 400, true );
  #endif
  
  #ifdef DEBUG
    Serial.println( "Passed #if" );
  #endif
    mpu.initialize( );
  #ifdef DEBUG
    Serial.println( "Passed mpu initialize" );
  #endif
    devStatus = mpu.dmpInitialize( );
  #ifdef DEBUG
    Serial.println( "Passed devstatus" );
  #endif
  
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset( 0 );
    mpu.setYGyroOffset( 0 );
    mpu.setZGyroOffset( 0 );
    mpu.setZAccelOffset( 1688 ); // 1688 factory default for my test chip
  
  #ifdef DEBUG
    Serial.println( "Passed offsets" );
  #endif
    // make sure it worked (returns 0 if so)
    if ( devStatus == 0 )
    {
      mpu.setDMPEnabled( true );
  
      attachInterrupt( 0, dmpDataReady, RISING );
      mpuIntStatus = mpu.getIntStatus( );
  
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize( );
    }
}

void init_ESCs( void )
{
#ifdef ESC_CALIBRATION
	FL.InitHigh( );
	FR.InitHigh( );
	RL.InitHigh( );
	RR.InitHigh( );

	delay( 3000 );
#endif
	//delay( 1000 );

	FL.InitLow( );
	FR.InitLow( );
	RL.InitLow( );
	RR.InitLow( );

	delay( ESC_ARM_DELAY );
}

void init_PIDs( void )
{
	for ( int Q = 0; Q < PIDCount; Q++ )
	{
		PIDs[ Q ].SetMode( AUTOMATIC );
		PIDs[ Q ].SetSampleTime( 25 );
		PIDs[ Q ].SetOutputLimits( -255.0, 255.0 );
	}
}

void init_Radio( void )
{
	Radio.begin( );
	//Radio.enableAckPayload( );
	Radio.openReadingPipe( 1, RADIO_PIPE );
	Radio.startListening( );
}

void init_Propellers( void )
{
	FL.Attach( Pin_FL );
	FR.Attach( Pin_FR );
	RL.Attach( Pin_RL );
	RR.Attach( Pin_RR );

	delay( 100 );

	init_ESCs( );
}

void init_Gyro( void )
{
	SetGyroValues( );
}
