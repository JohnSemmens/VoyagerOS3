#include "sim_weather.h"
#include "AP_Math.h"

void sim_weather::init()
{
	WindSpeed = 10; // knots
	WindDirection = 350; // NNW
}


void sim_weather::update()
{
	// called in a 1 minute
	
	// V1.1 8/1/2022 added random component to wind direction
	 WindDirection = 350 + random(-15,16) ; // NNW + random component
}
