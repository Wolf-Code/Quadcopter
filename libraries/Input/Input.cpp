#include "Input.h"

Input::Input( const byte& Pin )
{
	pinMode( Pin, INPUT );
	this->Pin = Pin;
}

bool Input::ReadBool( void )
{
	return digitalRead( this->Pin ) == HIGH;
}

int Input::ReadAnalog( void )
{
	return analogRead( this->Pin );
}

float Input::ReadAnalogFloat( void )
{
	return float( this->ReadAnalog( ) / 1023.0 );
}


