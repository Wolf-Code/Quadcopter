#include "Propeller.h"
#define Prop_Min 850
#define Prop_Max 2150

void Propeller::Attach( byte Pin )
{
	this->Srv.attach( Pin );
}


void Propeller::SetThrottle( byte Amount )
{
	int Speed = map( Amount, 0, 255, Prop_Min, Prop_Max );
	this->Srv.writeMicroseconds( Speed );
}

void Propeller::InitHigh( void )
{
	this->SetThrottle( 255 );
}

void Propeller::InitLow( void )
{
	this->SetThrottle( 0 );
}

Propeller::~Propeller( )
{
}
