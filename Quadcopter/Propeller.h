#pragma once
#ifndef __PROPELLER_H__
#define __PROPELLER_H__
#include <Servo.h>
#include "Arduino.h"

class Propeller
{
public:
	~Propeller( );
	void SetThrottle( byte Amount );
	void Attach( byte Pin );
	void InitHigh( void );
	void InitLow( void );

private:
	Servo Srv;
};

#endif
