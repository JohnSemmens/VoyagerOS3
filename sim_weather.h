#pragma once
class sim_weather
{
public:
	int WindSpeed; // knots
	int WindDirection; // degrees. 0 degrees is a northerly wind

	void init();
	void update();
};

