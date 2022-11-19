// WindAngle_2950.h

#ifndef _WindAngle_mpu2950_t3_h
#define _WindAngle_mpu2950_t3_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "MPU9250_t3.h"

class WindAngle_9250
{

public:
	uint8_t I2CPort;
	MPU9250 *WingSailAngleSensor = new MPU9250();		// The IMU including Magnetic Compass and Gyro 

	int pitch;
	int roll;
	float temperature;
	int MagneticBearing;
	bool status;

	void Init(uint8_t I2CPort);
	void Read();

	void IMU_Mag_Init();
	void IMU_Mag_Calibrate();

	// load the config values relating to the IMU, into the IMU.
	void Load_IMU_Calibation_Values();

private:


};

#endif //_WindAngle_mpu2950_t3_h

