#include <SPI.h>
#include <RF24_config.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include <PID_v1.h>
#include "Arduino.h"
#include "Servo\Servo.h"
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include <Wire.h>
#endif

#include "Propeller.h"
#include "Gyroscope.h"

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
Gyroscope* Gyro;

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
	Gyro->Loop( );

	Yaw = double( Gyro->GetYaw( ) );
	Pitch = double( Gyro->GetPitch( ) );
	Roll = double( Gyro->GetRoll( ) );

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