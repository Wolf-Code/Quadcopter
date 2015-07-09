#pragma once
#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include "Arduino.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

class Gyroscope
{
public:
	Gyroscope( bool AddressHigh );
	void Loop( void );
	Quaternion GetQuaternion( void );
	float* GetEuler( void );
	float* GetYawPitchRoll( void );
	float GetPitch( void );
	float GetYaw( void );
	float GetRoll( void );
	VectorFloat GetGravity( void );
	VectorInt16 GetAcceleration( void );
	VectorInt16 GetGravityFreeAcceleration( void );
	VectorInt16 GetWorldFrameAcceleration( void );

	~Gyroscope( );

private:
	MPU6050 mpu;
	// MPU control/status vars
	bool dmpReady;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[ 64 ]; // FIFO storage buffer

	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[ 3 ];       // [psi, theta, phi]    Euler angle container
	float ypr[ 3 ];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};

#endif