#ifndef __INPUT_H__
#define __INPUT_H__

#include <Arduino.h>

class Input
{
public:
	Input( const byte& Pin );

	bool ReadBool( void );
	int ReadAnalog( void );
	float ReadAnalogFloat( void );

private:
	byte Pin;
};

#endif