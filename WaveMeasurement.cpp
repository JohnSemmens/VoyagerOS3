// 
// 
// 

#include "WaveMeasurement.h"

void WaveClass::init(float Baro)
{
	Serial.println(F("*** Initialising Wave Class ..."));

	// set all pressure values to the current Barometric pressure to initialise
	Last_SLPressure = Baro;
	Last_LowPressure = Baro;
	Last_HighPressure = Baro; 
	Last_FilteredBaro = Baro;

	SLPressure = Baro;
	LowPressure = Baro;
	HighPressure = Baro;
	FilteredBaro = Baro;

	char MsgString[16];
	Serial.print(F("Initialised using: "));
	Serial.print(dtostrf(Baro, 7, 2, MsgString));
	Serial.println(F("hPa"));

	Serial.println(F("*** Wave Class Initialised ..."));
}

void WaveClass::update(float Baro)
{
  // called in the fast measurement loop // 200ms

	//SLPressure; //Pressure at sea level hPa
	// low pass filtered Baro with slow filter constant. This is to establish the mean sea level air pressure
	SLPressure = Last_SLPressure + filterConstant_slow * (Baro - Last_SLPressure);
	Last_SLPressure = SLPressure;

	// low pass filtered Baro with FAST filter constant. This is to establish the cleaned up version of current Barometric Pressure.
	// it will be used for zero crossing detection to work out the wave period.
	FilteredBaro = Last_FilteredBaro + filterConstant_fast * (Baro - Last_FilteredBaro);

	// perform rising edge, zero-crossing detection 
	// look for current baro to be above average line, while previous is below.
	if ((FilteredBaro > SLPressure) && (Last_FilteredBaro < SLPressure))
	{
		// record time
		uint32_t ms = millis();
		period = (ms - prev_ms)/1000.0; // convert to  seconds
		prev_ms = ms;
	}
	Last_FilteredBaro = FilteredBaro;


	// peak detection and hold for the high pressure part of the wave (wave trough)
	// low pass filter is used for the allowing the "held" value to fade away
	if (Last_HighPressure < Baro)
	{
		Last_HighPressure = Baro;
	}
	else 
	{
		HighPressure = Last_HighPressure + filterConstant_slow * (Baro - Last_HighPressure);
		Last_HighPressure = HighPressure;
	}


	// peak detection and hold for the low  pressure part of the wave (wave peak)
	// low pass filter is used for the allowing the "held" value to fade away
	if (Last_LowPressure > Baro)
	{
		Last_LowPressure = Baro;
	}
	else
	{
		LowPressure = Last_LowPressure + filterConstant_slow * (Baro - Last_LowPressure);
		Last_LowPressure = LowPressure;
	}


	//height; // wave height in metres  Peak - Trough
	// also subtract 2 x LSD (least significant digit)
	height = (HighPressure - LowPressure - 0.02) / hPa_per_m; // at sea level 15 deg C, -0.12 hPa/m

	// clamp the wave height to a positive value
	if (height < 0)
		height = 0;

}