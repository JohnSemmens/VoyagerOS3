#pragma once
class sim_weather
{
public:
	int WindSpeed; // knots
	int WindDirection; // degrees. 0 degrees is a northerly wind
	int MajorWindDirection;

	void init();
	void update();
};

