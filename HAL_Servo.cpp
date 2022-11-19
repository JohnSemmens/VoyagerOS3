// 
// 
// 

#include "HAL_Servo.h"
#include "HAL.h"
#include <Servo.h>

extern HALServo servo;

void HALServo::Servo_Out(int PulseWidth)
{
	// output a value to a servo.
	// V1.0 13/10/2016 John Semmens
	// V1.1 19/6/2022 updated and placed in a class

	// check that the request is within resonable bounds before writing to the servo.
	if ((RC_PulseWidth_Min <= PulseWidth) && (PulseWidth <= RC_PulseWidth_Max)) {
		Servo_Channel.writeMicroseconds(PulseWidth);
		ServoPulseWidth = PulseWidth;

		if (abs(prev_ServoPulseWidth - ServoPulseWidth) >= PowerOffDeadband)
		{
			// servo on 
			digitalWrite(Servo_Ch0_PWR_PIN, LOW);
			PoweredOn = true;
		}
	}
};

void HALServo::Servos_Init(void)
{
	Servo_Channel.attach(Servo_Ch0_OUT_PIN);
	
	// power on
	pinMode(Servo_Ch0_PWR_PIN, OUTPUT);
	digitalWrite(Servo_Ch0_PWR_PIN, LOW);
	PoweredOn = true;

	// set to failsafe value
	Servo_Channel.write(1500);

	PowerOffDeadband = 10; // us
}

void HALServo::PowerManagement()
{
	// called in the one second loop
	if (abs(prev_ServoPulseWidth - ServoPulseWidth) < PowerOffDeadband)
	{
		// servo off 
		digitalWrite(Servo_Ch0_PWR_PIN, HIGH);
		PoweredOn = false;
	}
	prev_ServoPulseWidth = ServoPulseWidth;
}