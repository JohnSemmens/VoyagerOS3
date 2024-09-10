#include "sim_weather.h"
#include "AP_Math.h"

void sim_weather::init()
{
	WindSpeed = 10; // knots
	WindDirection = 350; // NNW
	MajorWindDirection = 350;
}


void sim_weather::update()
{
	// called in a 1 minute
	
	// V1.1 8/1/2022 added random component to wind direction
	 WindDirection = MajorWindDirection + random(-15,16) ; // NNW + random component
}
