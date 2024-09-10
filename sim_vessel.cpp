#include "location.h"
#include "sim_vessel.h"
#include "AP_Math.h"
#include "configValues.h"
#include "sim_weather.h"

extern configValuesType Configuration;		// stucture holding Configuration values; preset variables
extern double SteeringServoOutput_LPF;
extern sim_weather simulated_weather;

void sim_vessel::init()
{
}

void sim_vessel::update()
{
	// update the simulated vessel position and attitude 
	// // called in a 1 second loop
	// V1.1 8/1/2022 added random component to heading update.
	
	// maintain an elapsed time between updates.
	unsigned long current_time = millis();
	update_time_ms = current_time - prev_update_time;
	prev_update_time = current_time;

	int windAngle;

	if (update_time_ms < 10000) // < 10seconds
	{
		windAngle = wrap_180(simulated_weather.WindDirection - Heading);// -ve is port tack, +ve starboard tack

		int SimTrueWindError = 40;

		// add an error component which is related to the tack
		if (windAngle > 0)
			windAngle = windAngle - SimTrueWindError; // eg. 160
		else
			windAngle = windAngle + SimTrueWindError; // eg. 190

		// calculate the simulated distance moved based on the VPP and elapsed time since last update
		SOG_mps = vpp( windAngle, simulated_weather.WindSpeed);
		float UpdateDistance = SOG_mps * update_time_ms / 1000;

		// calculate the simulated heading change based on rudder position, and SOG (sort of boat speed) and elapsed time since last update
		float TurnRateFactor = 40; // 30; // degrees/second
		HeadingChange = ((SteeringServoOutput_LPF - Configuration.pidCentre) / 500.0) * SOG_mps / 1.0 * (update_time_ms / 1000.0) * TurnRateFactor; // update heading, in degrees

		// Add a random component to the heading
		RandomHeadingComponent = random(-3, 4);
	
		Heading = wrap_360_Int(Heading - HeadingChange + RandomHeadingComponent);


		// update the simulated vessel postion  based on new heading and distance.
		location_update(Currentloc, (float)Heading, UpdateDistance);	
		
		if (windAngle > 0) // simulatethe real wing angle in response to apparent wind.
			WingsailAngle = windAngle - 15; 
		else
			WingsailAngle = windAngle + 15;

		// fall way if too high < 20 degrees
		if (abs(windAngle) < 20)
		{
			if (windAngle < 0) // port tack
			{ //port tack
				Heading = wrap_360_Int(Heading + (20 + windAngle)); // increase heading, on  if too close to wind
			}
			else
			{ // Starboard tack
				Heading = wrap_360_Int(Heading + (-20 + windAngle)); // decrease heading
			}
		}
	}
}

float sim_vessel::vpp(int windAngle, int windspeed) // degrees from head-to-wind, knots
{
	// V1.0 9/4/2021 initial simple VPP program
	
	int AbsWindAngle = abs(windAngle);
	float velocity; // Metres/second

	// 10 knot wind at 90 degrees yields 1 m/s boat speed.

	if (AbsWindAngle <= 90) // 0 to 90 degrees, increase speed proportionally with angle
	{
		velocity = AbsWindAngle / 90.0 * windspeed / 10.0;
	}
	else
	{
		velocity =  windspeed / 10.0; // speed is constant in lower half of polar diagram.
	}

	return velocity;
}