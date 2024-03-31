// 
// 
// 

#include "HAL.h"
#include "MagneticSensorLsm303.h"
#include "HAL_IMU.h"
//#include "i2c_driver_wire.h"  // Teensy 4.1 I2C driver
#include "i2c_t3.h"


#include "DisplayStrings.h"
#include "configValues.h"

extern configValuesType Configuration;
//extern I2CDriverWire* Wires[3]; // array for I2C wire 

void HALIMU::Init()
{
    Serial.println(F("*** Initialising IMU - LSM303..."));
    EquipmentStatus = EquipmentStatusType::Unknown;

    compass.I2CPort = 0;
    compass.init();

    //	compass.magnetometer_min = (MagneticSensorLsm303::vector<int16_t>){ -32767, -32767, -32767 };
    //	compass.magnetometer_max = (MagneticSensorLsm303::vector<int16_t>){ +32767, +32767, +32767 };

    compass.magnetometer_min = (MagneticSensorLsm303::vector<int16_t>)
    { Configuration.CompassMinX, Configuration.CompassMinY, Configuration.CompassMinZ };
    compass.magnetometer_max = (MagneticSensorLsm303::vector<int16_t>)
    { Configuration.CompassMaxX, Configuration.CompassMaxY, Configuration.CompassMaxZ };

    compass.enable();
    compass.EnableCalibration(false);
    if (compass.DeviceOk)
        EquipmentStatus = EquipmentStatusType::Found;
    else
        EquipmentStatus = EquipmentStatusType::NotFound;

    Serial.print(F("IMU status: "));
    Serial.println(GetEquipmentStatusString(EquipmentStatus));

    Serial.println(F("*** Initialising IMU complete."));
    Serial.println();
}

void HALIMU::Read()
{
    compass.read();

    Heading = wrap_360_Int(compass.getNavigationAngle() - Configuration.CompassOffsetAngle);
    Pitch = -1 * compass.pitch;
    Roll = compass.roll - Configuration.RollEror;

    TemperatureC = compass.readTempC();
}


