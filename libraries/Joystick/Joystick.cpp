#include "Joystick.h"

Joystick::Joystick( const byte& XPin, const byte& YPin, const byte& ButtonPin ) : XIn( XPin ), YIn( YPin ), ButtonIn( ButtonPin )
{

}

int Joystick::X( void )
{
	return this->XIn.ReadAnalog( );
}

int Joystick::Y( void )
{
	return this->YIn.ReadAnalog( );
}

byte Joystick::XByte( void )
{
	return byte( this->XIn.ReadAnalog( ) / 1023.0 );
}

byte Joystick::YByte( void )
{
	return byte( this->YIn.ReadAnalog( ) / 1023.0 );
}

float Joystick::XFloat( void )
{
	return this->XIn.ReadAnalogFloat( );
}

float Joystick::YFloat( void )
{
	return this->YIn.ReadAnalogFloat( );
}
