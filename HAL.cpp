// 
// 
// 

#include "HAL.h"
#include "i2c_t3.h"

void Scan_I2C_Buses()
{
	Serial.println(F("*** Scanning I2C Bus 0"));
	i2c_t3(0).I2Cscan();

	Serial.println(F("*** Scanning I2C Bus 1"));
	i2c_t3(1).I2Cscan();

	Serial.println(F("*** Scanning I2C Bus 2"));
	i2c_t3(2).I2Cscan();

	Serial.println(F("*** Scanning I2C Buses complete"));
	Serial.println();
}