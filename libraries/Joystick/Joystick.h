#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include <Arduino.h>
#include <Input.h>

class Joystick
{
public:
	Joystick( const byte& XPin, const byte& YPin, const byte& ButtonPin );
	int X( void );
	int Y( void );
	byte XByte( void );
	byte YByte( void );
	float XFloat( void );
	float YFloat( void );

private:
	Input XIn, YIn, ButtonIn;
};

#endif