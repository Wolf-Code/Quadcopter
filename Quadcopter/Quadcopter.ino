#include <SPI.h>
#include <RF24_config.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include <PID_v1.h>
#include "Arduino.h"
#include "Servo\Servo.h"

#define Pin_FL A0
#define Pin_FR A1
#define Pin_RL A2
#define Pin_RR A3

#define CE_PIN  9
#define CSN_PIN 10

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>

#include <MPU6050_6Axis_MotionApps20.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#include "Propeller.h"
#include "Gyroscope.h"

Propeller FL, FR, RL, RR;
Gyroscope* Gyro;

double Pitch, Yaw, Roll;

double Throttle_Pitch_Forward;
double Throttle_Pitch_Backwards;
double Throttle_Yaw_Forward;
double Throttle_Yaw_Backwards;
double Throttle_Roll_Forward;
double Throttle_Roll_Backwards;

double SetPoint_Pitch = 0;
double SetPoint_Yaw = 0;
double SetPoint_Roll = 0;

double consKp = 0.8, consKi = 0.04, consKd = 0.01;
PID PID_Pitch( &Pitch, &Throttle_Pitch_Forward, &SetPoint_Pitch, consKp, consKi, consKd, REVERSE );
PID PID_Pitch_Backwards( &Pitch, &Throttle_Pitch_Backwards, &SetPoint_Pitch, consKp, consKi, consKd, DIRECT );

PID PID_Roll( &Roll, &Throttle_Roll_Forward, &SetPoint_Roll, consKp, consKi, consKd, REVERSE );
PID PID_Roll_Backwards( &Roll, &Throttle_Roll_Backwards, &SetPoint_Roll, consKp, consKi, consKd, DIRECT );

#define PIDCount 4
PID PIDs[ PIDCount ] = { PID_Pitch, PID_Pitch_Backwards, PID_Roll, PID_Roll_Backwards };

RF24 Radio( CE_PIN, CSN_PIN );
// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe

// [ 1, 2 ] [ 3, 4 ] = Left stick, right stick.
byte Joystick[ 4 ];
float Joystick_Float[ 4 ];

#define Divider_Correction 2.5
#define ESC_Max 0.4
#define ESC_ARM_DELAY 3000

#define DEBUG
//#define DEBUG_ANGLES
//#define DEBUG_THROTTLE
#define DEBUG_JOYSTICKS
//#define ESC_CALIBRATION

long LastRadioLoss;
#define RadioLossPeriod 500

void setup( )
{
	
#ifdef DEBUG
	Serial.begin( 115200 );    // start serial at 9600 baud
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

	Gyro = new Gyroscope( false );

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
	float Front = ( Throttle_Pitch_Forward - Throttle_Pitch_Backwards );
	float Rear = ( Throttle_Pitch_Backwards - Throttle_Pitch_Forward );

	float Left = ( Throttle_Roll_Forward - Throttle_Roll_Backwards );
	float Right = ( Throttle_Roll_Backwards - Throttle_Roll_Forward );

	float Throttle = Joystick_Float[ 1 ];

	float tFL = constrain( Throttle + ( Front + Left ) / Divider_Correction, 0, ESC_Max );
	float tFR = constrain( Throttle + ( Front + Right ) / Divider_Correction, 0, ESC_Max );
	float tRL = constrain( Throttle + ( Rear + Left ) / Divider_Correction, 0, ESC_Max );
	float tRR = constrain( Throttle + ( Rear + Right ) / Divider_Correction, 0, ESC_Max );

	FL.SetThrottle( byte( tFL * 255 ) );
	FR.SetThrottle( byte( tFR * 255 ) );
	RL.SetThrottle( byte( tRL * 255 ) );
	RR.SetThrottle( byte( tRR * 255 ) );
#ifdef DEBUG_THROTTLE
	Serial.print( "FL: " );
	Serial.println( byte( tFL * 255 ) );
	Serial.print( "FR: " );
	Serial.println( byte( tFR * 255 ) );
	Serial.print( "RL: " );
	Serial.println( byte( tRL * 255 ) );
	Serial.print( "RR: " );
	Serial.println( byte( tRR * 255 ) );
	Serial.println( );
#endif
}

void UpdatePIDs( )
{
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
		LastRadioLoss = millis( );
	}

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
	Gyro->Loop( );

	Yaw = double( Gyro->GetYaw( ) );
	Pitch = double( Gyro->GetPitch( ) );
	Roll = double( Gyro->GetRoll( ) );

#ifdef DEBUG_ANGLES
	Serial.print( "Pitch: " );
	Serial.println( Pitch );
	Serial.print( "Yaw: " );
	Serial.println( Yaw );
	Serial.print( "Roll: " );
	Serial.println( Roll );
	Serial.println( );
#endif
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
		PIDs[ Q ].SetOutputLimits( 0, 255 );
	}
}

void init_Radio( void )
{
	Radio.begin( );
	Radio.openReadingPipe( 1, pipe );
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