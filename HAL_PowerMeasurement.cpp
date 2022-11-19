// 
// 
// 

#include "HAL_PowerMeasurement.h"
#include "i2c_t3.h"
#include "SDL_Arduino_INA3221.h"

SDL_Arduino_INA3221 ina3221;

void HALPowerMeasure::init()
{
	Serial.println(F("*** Initialising V/I Measurement..."));
    EquipmentStatus = EquipmentStatusType::Unknown;

    ina3221.begin();

    Serial.print("Manufacturer's ID=0x");
    int MID = ina3221.getManufID();
    Serial.println(MID, HEX);

    if (MID > 0) {
        EquipmentStatus = EquipmentStatusType::Found;
    }


	Serial.println(F("*** Initialising V/I Measurement complete."));
    Serial.println();
}


void HALPowerMeasure::read()
{
    Solar_V = ina3221.getBusVoltage_V(Solar);
    Solar_I = ina3221.getCurrent_mA(Solar);

    BatteryIn_V = ina3221.getBusVoltage_V(BatteryIn);
    BatteryIn_I = ina3221.getCurrent_mA(BatteryIn);

    BatteryOut_V = ina3221.getBusVoltage_V(BatteryOut);
    BatteryOut_I =  -1 * ina3221.getCurrent_mA(BatteryOut);
}

