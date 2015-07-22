/*
http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
1 - GND
2 - VCC 3.3V !!! NOT 5V
3 - CE to Arduino pin 9
4 - CSN to Arduino pin 10
5 - SCK to Arduino pin 13
6 - MOSI to Arduino pin 11
7 - MISO to Arduino pin 12
8 - UNUSED*/

#include <Joystick.h>
#include <Input.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define X_Left A3
#define Y_Left A2

#define X_Right A1
#define Y_Right A0

#define CE_PIN  9
#define CSN_PIN 10

// NOTE: the "LL" at the end of the constant is "LongLong" type
#define RADIO_PIPE 0xABCDABCD71LL // Define the transmit pipe

Joystick Left( X_Left, Y_Left, 6 ), Right( X_Right, Y_Right, 7 );

RF24 radio( CE_PIN, CSN_PIN ); // Create a Radio
byte joystick[ 4 ];  // 2 element array holding Joystick readings

//#define DEBUG

void setup()
{
#ifdef DEBUG
	Serial.begin( 115200 );
#endif
	init_Radio( );
	pinMode( Y_Right, INPUT );
}

void loop( )
{
	byte B = map( analogRead( Y_Right ), 0, 1023, 0, 255 );
	
	joystick[ 0 ] = Left.X( );
	joystick[ 1 ] = Left.Y( );
	joystick[ 2 ] = Right.X( );
	joystick[ 3 ] = B;//Right.Y( );

	radio.write( joystick, sizeof( joystick ) );

#ifdef DEBUG
	/*Serial.print( "Left:\tX=" );
	Serial.print( Left.X( ) );
	Serial.print( ", Y=" );
	Serial.println( Left.Y( ) );

	Serial.print( "Right:\tX=" );
	Serial.print( Right.X( ) );
	Serial.print( ", Y=" );
	Serial.println( Right.Y( ) );*/
	Serial.println( B );
#endif
}

void init_Radio( void )
{
	radio.begin( );
	//radio.enableAckPayload( );
	//radio.setAutoAck( false );
	radio.openWritingPipe( RADIO_PIPE );
}
