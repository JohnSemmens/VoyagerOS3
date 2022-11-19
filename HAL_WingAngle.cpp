// 
// 
// wingsail angle in degrees; positive angles to starboard, negative angles to port: -180 to 0 to +180

#include "HAL_WingAngle.h"
#include "configValues.h"
#include "DisplayStrings.h"

extern configValuesType Configuration;

void HALWingAngle::Init(void)
{
	Serial.println(F("*** Initialising WingAngle Sensors..."));

	Serial.println(F("Port WingAngle Sensor..."));
	WingSailAngleSensorPort.Init(1); // I2c #1

	PortStatus = EquipmentStatusType::Unknown;
	if (WingSailAngleSensorPort.status)
		PortStatus = EquipmentStatusType::Found;

	Serial.print(F("Port WingAngle Sensor status: "));
	Serial.println(GetEquipmentStatusString(PortStatus));
	Serial.println(F("Port WingAngle Sensor Initialising complete."));

	Serial.println(F("Starboard WingAngle Sensor..."));
	WingSailAngleSensorStbd.Init(2); // I2c #2

	StbdStatus = EquipmentStatusType::Unknown;
	if (WingSailAngleSensorStbd.status)
		StbdStatus = EquipmentStatusType::Found;

	Serial.print(F("Starboard WingAngle Sensor status: "));
	Serial.println(GetEquipmentStatusString(StbdStatus));
	Serial.println(F("Starboard WingAngle Sensor Initialising complete."));

	Read();
	PeakAnglePort = Angle;
	PeakAngleStbd = Angle;

	Serial.println(F("*** Initialising WingAngle Sensors complete."));
	Serial.println();
}

void HALWingAngle::Read()
{
	WingSailAngleSensorPort.Read();
	WingSailAngleSensorStbd.Read();

	// use the port sensor and connections.
	int RawAngle = WingSailAngleSensorPort.MagneticBearing + Configuration.WindAngleCalibrationOffset;

	Deviation = DeviationCalc(RawAngle); // Deviation Error
	Angle = wrap_180(RawAngle - Deviation); // subtract the Deviation Error

	PortTemperature = WingSailAngleSensorPort.temperature;
	StbdTemperature = WingSailAngleSensorStbd.temperature;

	// test code for sending mx and my to USB serial.
	// only send out serial data while the OLED Display is showing the detailed Wingsail Sensor data.
	if (Configuration.DisplayScreenView == 'y')
	{
		Serial.print(WingSailAngleSensorPort.WingSailAngleSensor->mx);
		Serial.print(",");
		Serial.print(WingSailAngleSensorPort.WingSailAngleSensor->my);
		Serial.println();
	}
}


int HALWingAngle::DeviationCalc(int MagneticAngle)
{
	// function to provide a simple interpolated correction for the magnetic Angle returned by the WingAngle Sensor.
	// This provides an error value to be subracted from the Wing Angle.
	// Currently this is simply a linear interpolation. Maybe a Cubic spline is better. Maybe it doesn't matter.
	// Currently their is only support for one wing angle sensor.

	// V1.0 17/7/2021 John Semmens

	// these will ultimately be stored in config
	//int WingAngleError000 =  +2;
	//int WingAngleError090 =  -9;
	//int WingAngleError180 =  -4;
	//int WingAngleError270 =   0;

	float InterpolatedError=0;

	//stbd bow -- ok
	if (MagneticAngle >= 0 && MagneticAngle < 90)
	{
		InterpolatedError = (MagneticAngle-0) * (Configuration.WingAngleError090 - Configuration.WingAngleError000)/(90) + Configuration.WingAngleError000;
	}

	// stbd qtr -- ok
	if (MagneticAngle >= 90 && MagneticAngle < 180)
	{
		InterpolatedError = (MagneticAngle-90) * (Configuration.WingAngleError180 - Configuration.WingAngleError090)/(90) + Configuration.WingAngleError090;
	}

	//port bow -- ok
	if (MagneticAngle < 0 && MagneticAngle > -90)
	{
		InterpolatedError = (MagneticAngle-0) * (Configuration.WingAngleError270 - Configuration.WingAngleError000)/(-90) + Configuration.WingAngleError000;
	}

	//port qtr -- ok
	if (MagneticAngle <= -90 && MagneticAngle > -180)
	{
		InterpolatedError = (MagneticAngle+90) * (Configuration.WingAngleError180 - Configuration.WingAngleError270)/(-90) + Configuration.WingAngleError270;
	}

	return InterpolatedError;
}


void HALWingAngle::UpdateMovementDetection(int WingSailAngle)
{

	// positive angles to starboard, negative angles to port

	// slowly drain away the peaks toward zero
	PeakAnglePort = (float)PeakAnglePort * 0.999;
	PeakAngleStbd = (float)PeakAngleStbd * 0.999;

	// establish instantaneous peaks in both directions
	if (WingSailAngle >= 0) // starboard +ve angle
	{
		if (PeakAngleStbd < WingSailAngle)			PeakAngleStbd = WingSailAngle;
		if (PeakAnglePort > (WingSailAngle-360))	PeakAnglePort = WingSailAngle - 360;
	}
	else // port -ve angles
	{
		if (PeakAnglePort > WingSailAngle)			PeakAnglePort = WingSailAngle;
		if (PeakAngleStbd < (WingSailAngle+360))	PeakAngleStbd = WingSailAngle + 360;
	}

	Movement = PeakAngleStbd - PeakAnglePort - 360;
}

void HALWingAngle::HealthCheck()
{
	// this is expected to the called every 10 seconds

	static int prev_Movement;

	if (Movement == 0 && prev_Movement == 0)
	{
		// fault
		PortStatus = EquipmentStatusType::DataBad;
	} 
	else
	{
		// Ok
		PortStatus = EquipmentStatusType::DataGood;
	}
	prev_Movement = Movement;
}