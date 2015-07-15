#include <SPI.h>
#include <RF24_config.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include <PID_v1.h>
#include "Arduino.h"
#include "Servo\Servo.h"

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

#define PI 3.14159

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

double SetPoint_Pitch = 0;
double SetPoint_Yaw = 0;
double SetPoint_Roll = 0;

#define PITCH_K 0.6
#define PITCH_I 0.15
#define PITCH_D 0.06

#define ROLL_K 0.4
#define ROLL_I 0.075
#define ROLL_D 0.06

#define YAW_K ROLL_K
#define YAW_I ROLL_I
#define YAW_D ROLL_D

#define PITCH_MAX 10
#define PITCH_MIN -10

#define ROLL_MAX 10
#define ROLL_MIN -10

#define YAW_MAX 180
#define YAW_MIN -180
#define YAW_INFLUENCE 0.1

PID PID_Pitch( &Pitch, &Throttle_Pitch, &SetPoint_Pitch, PITCH_K, PITCH_I, PITCH_D, REVERSE );
PID PID_Roll( &Roll, &Throttle_Roll, &SetPoint_Roll, ROLL_K, ROLL_I, ROLL_D, REVERSE );
PID PID_Yaw( &Yaw, &Throttle_Yaw, &SetPoint_Yaw, YAW_K, YAW_D, YAW_I, REVERSE );

#define PIDCount 2
PID PIDs[ PIDCount ] = { PID_Pitch, PID_Roll };

RF24 Radio( CE_PIN, CSN_PIN );
#define RADIO_PIPE 0xE8E8F0F0E1LL

// [ 1, 2 ] [ 3, 4 ] = Left stick, right stick.
byte Joystick[ 4 ];
float Joystick_Float[ 4 ];

#define CORRECTION_MAX 0.25
#define ESC_MAX 1
#define ESC_ARM_DELAY 3000
#define THROTTLE_MIN 0.1

//#define DEBUG
//#define DEBUG_ANGLES
//#define DEBUG_THROTTLE
//#define DEBUG_JOYSTICKS
//#define ESC_CALIBRATION

long LastRadioLoss;
#define RadioLossPeriod 500

Propeller FL, FR, RL, RR;
Propeller Props_CW[ ] = { FR, RL };
Propeller Props_CCW[ ] = { FL, RR };
Gyroscope* Gyro;

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
	float Throttle = Joystick_Float[ 3 ];

	if ( Throttle <= THROTTLE_MIN )
	{
		FL.SetThrottle( 0 );
		FR.SetThrottle( 0 );
		RL.SetThrottle( 0 );
		RR.SetThrottle( 0 );
		return;
	}

	float P = Throttle_Pitch;
#ifdef DEBUG_THROTTLE
	Serial.println( Throttle_Pitch );
	Serial.println( Throttle_Roll );
	Serial.println( Throttle_Yaw );
#endif
	float R = Throttle_Roll;
	float Y = Throttle_Yaw / 255.0;

	float tFL = constrain( P + R + YAW_INFLUENCE * Y, 0, CORRECTION_MAX );
	float tFR = constrain( P - R - YAW_INFLUENCE * Y, 0, CORRECTION_MAX );
	float tRL = constrain( -P + R - YAW_INFLUENCE * Y, 0, CORRECTION_MAX );
	float tRR = constrain( -P - R + YAW_INFLUENCE * Y, 0, CORRECTION_MAX );

	tFL = constrain( tFL + Throttle, 0, ESC_MAX );
	tFR = constrain( tFR + Throttle, 0, ESC_MAX );
	tRL = constrain( tRL + Throttle, 0, ESC_MAX );
	tRR = constrain( tRR + Throttle, 0, ESC_MAX );

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
	}

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
		PIDs[ Q ].SetOutputLimits( -255, 255 );
	}
}

void init_Radio( void )
{
	Radio.begin( );
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