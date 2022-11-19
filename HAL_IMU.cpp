// 
// 
// 
#include "config.h"
#include "HAL.h"
#include "HAL_IMU.h"

#include "config.h"
#include "i2c_t3.h"
#include "Alarms.h"
#include "I2Cdev.h"
#include "USFSMAX.h"
#include "Sensor_cal.h"
#include "IMU.h"
#include "Globals.h"
#include "Types.h"
#include "def.h"
#include "DisplayStrings.h"
#include "configValues.h"

extern configValuesType Configuration;

// Instantiate class objects
I2Cdev     i2c_0(&SENSOR_0_WIRE_INSTANCE);
USFSMAX    USFSMAX_0(&i2c_0, 0);
IMU        imu_0(&USFSMAX_0, 0);
Sensor_cal sensor_cal(&i2c_0, &USFSMAX_0, 0);

// Declare global scope utility functions
void       ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void       FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM);
void       DRDY_handler_0();
//void       SerialInterface_handler();
#define SERIAL_DEBUG


void HALIMU::Init()
{
	Serial.println(F("*** Initialising IMU..."));
    EquipmentStatus = EquipmentStatusType::Unknown;

    // Set up DRDY interrupt pin
    pinMode(INT_PIN, INPUT);

    // Assign Indicator LED
    LEDPIN_PINMODE;
    Alarms::blueLEDoff();

    // Initialize USFSMAX_0 I2C bus
    SENSOR_0_WIRE_INSTANCE.begin(I2C_MASTER, 0x00, I2C_PINS, I2C_PULLUP_EXT, I2C_CLOCK);
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_RATE_100);                                                                     // Set I2C clock speed to 100kHz cor configuration
    delay(2000);

    // Do I2C bus scan if serial debug is active
#ifdef SERIAL_DEBUG                                                                                                // Should see MAX32660 slave bus address (default is 0x57)
  //  i2c_0.I2Cscan();
#endif

    // Initialize USFSMAX_0
#ifdef SERIAL_DEBUG
    Serial.print("Initializing USFSMAX_0...");
    Serial.println("");
#endif
    USFSMAX_0.init_USFSMAX();                                             // Configure USFSMAX and sensors 
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                         // Set the I2C clock to high speed for run-mode data collection
    delay(100);

    // Attach interrupts
    attachInterrupt(INT_PIN, DRDY_handler_0, RISING);                   // Attach DRDY interrupt
#ifdef SERIAL_DEBUG
    Serial.println("USFXMAX_0 successfully initialized!");
   // sensor_cal.sendOneToProceed();                         // Halt the serial monitor to let the user read the results
#endif

// Calculate geomagnetic calibration parameters for your location (set in "config.h")
    Mv_Cal = M_V;                                                       // Vertical geomagnetic field component
    Mh_Cal = M_H;                                                      // Horizontal geomagnetic field component
    M_Cal = sqrt(Mv_Cal * Mv_Cal + Mh_Cal * Mh_Cal);                   // Geomagnetic field strength
    Del_Cal = atan(Mv_Cal / Mh_Cal);                                    // Geomagnetic inclination or "Dip" angle

    Start_time = micros();

    EquipmentStatus = USFSMAX_0.EquipmentStatus;
    Serial.print(F("IMU status: "));
    Serial.println(GetEquipmentStatusString(EquipmentStatus));

	Serial.println(F("*** Initialising IMU complete."));
    Serial.println();
}


void HALIMU::Read()
{
    // Calculate loop cycle time
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;

    if (data_ready[0] == 1)
    {
        data_ready[0] = 0;
        ProcEventStatus(&i2c_0, 0);                      // I2C instance 0, Sensor instance 0 (and implicitly USFSMAX instance 0)
        FetchUSFSMAX_Data(&USFSMAX_0, &imu_0, 0);        // USFSMAX instance 0, IMU calculation instance 0 and Sensor instance 0
    }

    // Update serial output
    delt_t = millis() - last_refresh;
    if (delt_t > UPDATE_PERIOD)                        // Update the serial monitor every "UPDATE_PERIOD" ms
    {
        last_refresh = millis();
        USFSMAX_0.GetMxMy();                           // Get Horizontal magnetic components

        // standard orientation
        if (Configuration.CompassOffsetAngle == 0) {
            Heading = heading[0];
            Pitch = angle[0][1];
            Roll = angle[0][0];
        }

        // rotated 90degrees -- for Voyager 2.5
        if (Configuration.CompassOffsetAngle == 90) {
            Heading = heading[0] + 90; // offset heading
            Pitch = -1 * angle[0][0]; // swap pitch and roll. Negate pitch
            Roll = angle[0][1];
        }

        Baro = ((float)baroADC[0]) / 4096.0f;
        Algorithm_Status = cal_status[0];

        // Toggle LED if not calibrating gyroscopes
        if (gyroCalActive[0] == 1)
        {
            Alarms::blueLEDoff();
            if ((i2c_0.readByte(MAX32660_SLV_ADDR, CALIBRATION_STATUS) & 0x01) == 0)
            {
                gyroCalActive[0] = 0;
            }
        }
        else
        {
            Alarms::toggle_blueLED();
        }
        data_ready[0] = 0;
    }

}


void ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM)
{
    uint8_t temp[1];

    // Read algorithm status and event status
    i2c_BUS->readBytes(MAX32660_SLV_ADDR, COMBO_DRDY_STAT, 1, temp);
    eventStatus[sensorNUM] = temp[0];

    // Decode the event status to determine what data is ready and set the appropriate DRDY fags
    if (eventStatus[sensorNUM] & 0x01) Gyro_flag[sensorNUM] = 1;
    if (eventStatus[sensorNUM] & 0x02) Acc_flag[sensorNUM] = 1;
    if (eventStatus[sensorNUM] & 0x04) Mag_flag[sensorNUM] = 1;
    if (eventStatus[sensorNUM] & 0x08) Baro_flag[sensorNUM] = 1;
    if (eventStatus[sensorNUM] & 0x10) Quat_flag[sensorNUM] = 1;
}


void FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM)
{
    uint8_t call_sensors = eventStatus[sensorNUM] & 0x0F;

    Acq_time = 0;
    Begin = micros();

    // Optimize the I2C read function with respect to whatever sensor data is ready
    switch (call_sensors)
    {
    case 0x01:
        usfsmax->GyroAccel_getADC();
        break;
    case 0x02:
        usfsmax->GyroAccel_getADC();
        break;
    case 0x03:
        usfsmax->GyroAccel_getADC();
        break;
    case 0x07:
        usfsmax->GyroAccelMagBaro_getADC();
        break;
    case 0x0B:
        usfsmax->GyroAccelMagBaro_getADC();
        break;
    case 0x0F:
        usfsmax->GyroAccelMagBaro_getADC();
        break;
    case 0x0C:
        usfsmax->MagBaro_getADC();
        break;
    case 0x04:
        usfsmax->MAG_getADC();
        break;
    case 0x08:
        usfsmax->BARO_getADC();
        break;
    default:
        break;
    };
    Acq_time += micros() - Begin;

    if (Mag_flag[sensorNUM])
    {
        if (ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                magData[sensorNUM][i] = ((float)magADC[sensorNUM][i]) * UT_per_Count;
            }
        }
        else                                                                                                           // Calibration data applied locally
        {
            sensor_cal.apply_adv_calibration(ellipsoid_magcal[sensorNUM], magADC[sensorNUM], UT_per_Count, mag_calData[sensorNUM]);
            sensor_cal.apply_adv_calibration(final_magcal[sensorNUM], mag_calData[sensorNUM], 1.0f, sensor_point);
            MAG_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
        }
        Mag_flag[sensorNUM] = 0;
    }
    if (Acc_flag[sensorNUM])
    {
        if (ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                accData[sensorNUM][i] = ((float)accADC[sensorNUM][i]) * g_per_count;
            }
        }
        else                                                                                                           // Calibration data applied locally
        {
            sensor_cal.apply_adv_calibration(accelcal[sensorNUM], accADC[sensorNUM], g_per_count, sensor_point);
            ACC_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
        }
        Acc_flag[sensorNUM] = 0;
    }
    if (Gyro_flag[sensorNUM] == 1)
    {
        if (ScaledSensorDataFlag)                                                                                         // Calibration data is applied in the coprocessor; just scale
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                gyroData[sensorNUM][i] = ((float)gyroADC[sensorNUM][i]) * dps_per_count;
            }
        }
        else                                                                                                           // Calibration data applied locally
        {
            sensor_cal.apply_adv_calibration(gyrocal[sensorNUM], gyroADC[sensorNUM], dps_per_count, sensor_point);
            GYRO_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
        }

        // Call alternative (Madgwick or Mahony) IMU fusion filter
        IMu->compute_Alternate_IMU();
        Gyro_flag[sensorNUM] = 0;
    }
    if (Quat_flag[sensorNUM] == 1)
    {
        IMu->computeIMU();
        Quat_flag[sensorNUM] = 0;
    }
}

// Host DRDY interrupt handler
void DRDY_handler_0()
{
    data_ready[0] = 1;
}



