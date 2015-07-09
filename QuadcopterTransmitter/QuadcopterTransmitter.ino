#include <Joystick.h>
#include <Input.h>
#define V_Out_Left 2
#define V_Out_Right 3

#define X_Left A4
#define Y_Left A6

#define X_Right A5
#define Y_Right A7

Joystick Left( X_Left, Y_Left, 9 ), Right( X_Right, Y_Right, 9 );

void setup()
{

  /* add setup code here */
	pinMode( V_Out_Left, OUTPUT );
	pinMode( V_Out_Right, OUTPUT );

	digitalWrite( V_Out_Left, HIGH );
	digitalWrite( V_Out_Right, HIGH );

	Serial.begin( 115200 );
}

void loop()
{
	Serial.print( "Left:\tX=" );
	Serial.print( Left.X( ) );
	Serial.print( ", Y=" );
	Serial.println( Left.Y( ) );

	Serial.print( "Right:\tX=" );
	Serial.print( Right.X( ) );
	Serial.print( ", Y=" );
	Serial.println( Right.Y( ) );
}
