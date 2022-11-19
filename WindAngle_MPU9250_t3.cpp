// 
// 
// 
#include "WindAngle_mpu9250_t3.h"

/* MPU9250 Basic Example Code
by: Kris Winer
date: April 1, 2014
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.
Modified by Brent Wilkins July 19, 2016

Demonstrate basic MPU-9250 functionality including parameterizing the register
addresses, initializing the sensor, getting properly scaled accelerometer,
gyroscope, and magnetometer data out. Added display functions to allow display
to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
and the Teensy 3.1.

SDA and SCL should have external pull-up resistors (to 3.3V).
10k resistors are on the EMSENSR-9250 breakout board.

Hardware setup:
MPU9250 Breakout --------- Arduino
VDD ---------------------- 3.3V
VDDI --------------------- 3.3V
SDA ----------------------- A4
SCL ----------------------- A5
GND ---------------------- GND

Modified by John Semmens, June 2021, for the Voyager Sailing Drones Project
Updated for Teensy 3 with multile I2C bus support.
Added support for both MPU-9250 and MPU-9255

*/

//#include <Wire.h>
#include "i2c_t3.h"
#include "MPU9250_t3.h"
#include "location.h"

#include "configValues.h"

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins


extern configValuesType Configuration;


void WindAngle_9250::Init(uint8_t I2CPort)
{
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 22/11/2016 updated to improve handling where there's no IMU found.

	// TWBR = 12;  // 400 kbit/sec I2C speed
	WingSailAngleSensor->I2CPort = I2CPort;
	i2c_t3(I2CPort).begin();

	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(intPin, INPUT);
	digitalWrite(intPin, LOW);

	// enable a calibration update on each read.
	//WingSailAngleSensor->MagneticCompassCalibrationMode = true;

	// Read the WHO_AM_I register, this is a good test of communication
	Serial.print("I2C Bus: "); Serial.print(I2CPort);
	Serial.print(" Addr: 0x"); Serial.print(MPU9250_ADDRESS, HEX);
	Serial.print(" Who_Am_I is 0x"); 
	byte c = WingSailAngleSensor->readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	Serial.print(c, HEX);
//	Serial.print(" I should be "); Serial.println(0x71, HEX);
	if (c == 0x71) Serial.println(": MPU-9250");
	if (c == 0x73) Serial.println(": MPU-9255");

	if ((c == 0x71) || (c == 0x73))// WHO_AM_I 9250= 0x71, 9255=0x73
	{
		Serial.println(F("MSG,MPU925x is online..."));

		// Start by performing self test and reporting values
		WingSailAngleSensor->MPU9250SelfTest(WingSailAngleSensor->SelfTest);

		// Calibrate gyro and accelerometers, load biases in bias registers
			WingSailAngleSensor->calibrateMPU9250(WingSailAngleSensor->gyroBias, WingSailAngleSensor->accelBias);

			WingSailAngleSensor->initMPU9250();
		// Initialize device for active mode read of acclerometer, gyroscope, and
		// temperature
		Serial.println("MSG,MPU9250 initialized for active data mode....");

		// Read the WHO_AM_I register of the magnetometer, this is a good test of
		// communication
	//	byte d = WingSailAngleSensor->readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
		//Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
		//Serial.print(" I should be "); Serial.println(0x48, HEX);

		// Get magnetometer calibration from AK8963 ROM
		WingSailAngleSensor->initAK8963(WingSailAngleSensor->magCalibration);

	//	IMU_Mag_Init();
		status = true;

	} // if (c == 0x71) or 73
	else
	{
		Serial.print(": No IMU MPU925x found: 0x");
		Serial.println(c, HEX);
		status = false;
	}
}

void  WindAngle_9250::IMU_Mag_Init()
{
	// This is used as a prelude to calibration.
	// It is intended to be used once to initiate the compass calibration limits to safe values prior to a calibration run.
	// V1.0 15/8/2016 John Semmens

	Read(); // ensure that the IMU Mag values are reasonable before clearing and re-initiating the recorded limits

	Configuration.WingAngle_mXScale = abs(WingSailAngleSensor->mx);
	Configuration.WingAngle_mYScale = abs(WingSailAngleSensor->my);
}

void  WindAngle_9250::IMU_Mag_Calibrate()
{
	// This is called after each compass read during a calibration run.
	// The IMU mx, my & mz values follow an approximate sine wave (with an amplitude an offset to be calibated out).
	// This function will adjust the upper and lower limits of the sine curve of each axis.
	// it is intended that that vessel is placed manually into compass calibration mode, and then about two full turns are performed.
	// This should set resonable upper and lower limits for each axis.
	// These should then be recorded and set in the config stucture EEPROM.
	// V1.0 15/8/2016 John Semmens

	// extend calibration limits
	// increase scale value to be the peak of the value for that axis.
	// capture peak and hold
	if (Configuration.WingAngle_mXScale < abs(WingSailAngleSensor->mx)) Configuration.WingAngle_mXScale = abs(WingSailAngleSensor->mx);
	if (Configuration.WingAngle_mYScale < abs(WingSailAngleSensor->my)) Configuration.WingAngle_mYScale = abs(WingSailAngleSensor->my);
}

void  WindAngle_9250::Read()
{
	// If intPin goes high, all data registers have new data
	// On interrupt, check if data ready interrupt
	if (WingSailAngleSensor->readByte( MPU9250_ADDRESS, INT_STATUS) & 0x01)
	{
		WingSailAngleSensor->readAccelData(WingSailAngleSensor->accelCount);  // Read the x/y/z adc values
		WingSailAngleSensor->getAres();

		// Now we'll calculate the acceleration value into actual g's
		// This depends on scale being set
		WingSailAngleSensor->ax = (float)WingSailAngleSensor->accelCount[0] * WingSailAngleSensor->aRes; // - accelBias[0];
		WingSailAngleSensor->ay = (float)WingSailAngleSensor->accelCount[1] * WingSailAngleSensor->aRes; // - accelBias[1];
		WingSailAngleSensor->az = (float)WingSailAngleSensor->accelCount[2] * WingSailAngleSensor->aRes; // - accelBias[2];

		WingSailAngleSensor->readGyroData(WingSailAngleSensor->gyroCount);  // Read the x/y/z adc values
		WingSailAngleSensor->getGres();

		// Calculate the gyro value into actual degrees per second
		// This depends on scale being set
		WingSailAngleSensor->gx = (float)WingSailAngleSensor->gyroCount[0] * WingSailAngleSensor->gRes;
		WingSailAngleSensor->gy = (float)WingSailAngleSensor->gyroCount[1] * WingSailAngleSensor->gRes;
		WingSailAngleSensor->gz = (float)WingSailAngleSensor->gyroCount[2] * WingSailAngleSensor->gRes;

		WingSailAngleSensor->readMagData(WingSailAngleSensor->magCount);  // Read the x/y/z adc values
		WingSailAngleSensor->getMres();

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental
		// corrections
		// Get actual magnetometer value, this depends on scale being set
		WingSailAngleSensor->mx = (float)WingSailAngleSensor->magCount[0] * WingSailAngleSensor->mRes*WingSailAngleSensor->magCalibration[0] - WingSailAngleSensor->magbias[0];
		WingSailAngleSensor->my = (float)WingSailAngleSensor->magCount[1] * WingSailAngleSensor->mRes*WingSailAngleSensor->magCalibration[1] - WingSailAngleSensor->magbias[1];
		WingSailAngleSensor->mz = (float)WingSailAngleSensor->magCount[2] * WingSailAngleSensor->mRes*WingSailAngleSensor->magCalibration[2] - WingSailAngleSensor->magbias[2];
	
		float mx_cal = WingSailAngleSensor->mx / (Configuration.WingAngle_mXScale);
		float my_cal = WingSailAngleSensor->my / (Configuration.WingAngle_mYScale);
		
		
		MagneticBearing = atan2(my_cal, mx_cal) * RAD_TO_DEG; // swap to y over x to provide a 90 degree orientation

		// simple Pitch and Roll calculation. No Yaw.
		roll = atan2(WingSailAngleSensor->ax, WingSailAngleSensor->az)  * RAD_TO_DEG;
		pitch = atan2(WingSailAngleSensor->ay, WingSailAngleSensor->az) * RAD_TO_DEG;

	} // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

	WingSailAngleSensor->tempCount = WingSailAngleSensor->readTempData();  // Read the adc values
												// Temperature in degrees Centigrade
	temperature = ((float)WingSailAngleSensor->tempCount) / 333.87 + 21.0;

	if (WingSailAngleSensor->MagneticCompassCalibrationMode==true)
			IMU_Mag_Calibrate();
}

void  WindAngle_9250::Load_IMU_Calibation_Values()
{
	// load the config values relating to the IMU, into the IMU.
	// V1.0 27/8/2016 John Semmens

	//WingSailAngleSensor->mx_min = Configuration.mx_min;
	//WingSailAngleSensor->mx_max = Configuration.mx_max;
	//WingSailAngleSensor->my_min = Configuration.my_min;
	//WingSailAngleSensor->my_max = Configuration.my_max;
	//WingSailAngleSensor->mz_min = Configuration.mz_min;
	//WingSailAngleSensor->mz_max = Configuration.mz_max;
}