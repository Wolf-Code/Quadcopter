#include "Gyroscope.h"
#include <MPU6050.h>

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady( )
{
	mpuInterrupt = true;
}

Gyroscope::Gyroscope( bool AddressHigh = false ) : mpu( AddressHigh ? 0x69 : 0x68 )
{
	dmpReady = false;
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin( );
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup( 400, true );
	#endif
	
#ifdef DEBUG
	Serial.println( "Passed #if" );
#endif
	mpu.initialize( );
#ifdef DEBUG
	Serial.println( "Passed mpu initialize" );
#endif
	devStatus = mpu.dmpInitialize( );
#ifdef DEBUG
	Serial.println( "Passed devstatus" );
#endif

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset( 0 );
	mpu.setYGyroOffset( 0 );
	mpu.setZGyroOffset( 0 );
	mpu.setZAccelOffset( 1688 ); // 1688 factory default for my test chip

#ifdef DEBUG
	Serial.println( "Passed offsets" );
#endif
	// make sure it worked (returns 0 if so)
	if ( devStatus == 0 )
	{
		mpu.setDMPEnabled( true );

		attachInterrupt( 0, dmpDataReady, RISING );
		mpuIntStatus = mpu.getIntStatus( );

		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize( );
	}
}

Quaternion Gyroscope::GetQuaternion( void )
{
	return q;
}

float* Gyroscope::GetEuler( void )
{
	return euler;
}

float* Gyroscope::GetYawPitchRoll( void )
{
	return ypr;
}

float Gyroscope::GetPitch( void )
{
	return ypr[ 1 ];
}

float Gyroscope::GetYaw( void )
{
	return ypr[ 0 ];
}

float Gyroscope::GetRoll( void )
{
	return ypr[ 2 ];
}

VectorFloat Gyroscope::GetGravity( void )
{
	return gravity;
}

VectorInt16 Gyroscope::GetAcceleration( void )
{
	return aa;
}

VectorInt16 Gyroscope::GetGravityFreeAcceleration( void )
{
	mpu.dmpGetLinearAccel( &aaReal, &aa, &gravity );
	return aaReal;
}

VectorInt16 Gyroscope::GetWorldFrameAcceleration( void )
{
	mpu.dmpGetLinearAccel( &aaReal, &aa, &gravity );
	mpu.dmpGetLinearAccelInWorld( &aaWorld, &aaReal, &q );

	return aaWorld;
}

void Gyroscope::Loop( void )
{
	// if programming failed, don't try to do anything
	if ( !dmpReady ) return;

	// wait for MPU interrupt or extra packet(s) available
	while ( !mpuInterrupt && fifoCount < packetSize )
	{
		// other program behavior stuff here
		// .
		// .
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		// .
		// .
		// .
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus( );

	// get current FIFO count
	fifoCount = mpu.getFIFOCount( );
	// check for overflow (this should never happen unless our code is too inefficient)
	if ( ( mpuIntStatus & 0x10 ) || fifoCount == 1024 )
	{
		// reset so we can continue cleanly
		mpu.resetFIFO( );
#ifdef DEBUG
		Serial.println( F( "FIFO overflow!" ) );
#endif

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if ( mpuIntStatus & 0x02 )
	{
		// wait for correct available data length, should be a VERY short wait
		while ( fifoCount < packetSize ) fifoCount = mpu.getFIFOCount( );

		// read a packet from FIFO
		mpu.getFIFOBytes( fifoBuffer, packetSize );

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
		mpu.dmpGetQuaternion( &q, fifoBuffer );
		mpu.dmpGetAccel( &aa, fifoBuffer );
		mpu.dmpGetGravity( &gravity, &q );
		mpu.dmpGetEuler( euler, &q );
		mpu.dmpGetYawPitchRoll( ypr, &q, &gravity );
	}
}

Gyroscope::~Gyroscope( )
{
	detachInterrupt( 0 );
}
