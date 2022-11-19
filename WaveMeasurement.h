// WaveMeasurement.h

#ifndef _WaveMeasurement_h
#define _WaveMeasurement_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



class WaveClass
{
public:
	//float peak;   // metres +ve
	//float trough; // metres -ve

	float SLPressure; //Pressure at sea level hPa
	float LowPressure; //  peak of wave
	float HighPressure; // trough of wave.
	float FilteredBaro; //

	float period; // wave period in seconds
	float height; // wave height in metres  Peak - Trough
	void init(float Baro);
	void update(float Baro);

	float hPa_per_m = 0.12; // at sea level 15 deg C, -0.12 hPa/m
	float filterConstant_slow = 0.01;
	float filterConstant_fast = 0.1;

protected:
	float Last_SLPressure;
	float Last_LowPressure; //  peak of wave
	float Last_HighPressure; // trough of wave.
	float Last_FilteredBaro;
	uint32_t prev_ms;
};


#endif

