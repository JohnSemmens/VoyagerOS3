/*!
 * @file MagenticSensorLsm303.cpp
 * @brief DFRobot's Magentic Sensor
 * @n Header file for DFRobot's Magentic Sensor
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [carl](carl.xu@dfrobot.com)
 * @version  V1.0
 * @date  2016-07-11
 */
#include "MagneticSensorLsm303.h"
//#include <Wire.h>
//#include "i2c_driver_wire.h"  // Teensy 4.1 I2C driver
#include "i2c_t3.h"
#include <math.h>

//extern I2CDriverWire* Wires[3]; // array for I2C wire 

MagneticSensorLsm303::MagneticSensorLsm303(void)
{
  /*
  These values lead to an assumed magnetometer bias of 0.
  Use the Calibrate example program to determine appropriate values
  for your particular unit. The Heading example demonstrates how to
  adjust these values in your own sketch.
  */
  //Wire.begin();
  //magnetometer_min = (MagneticSensorLsm303::vector<int16_t>){-32767, -32767, -32767};
  //magnetometer_max = (MagneticSensorLsm303::vector<int16_t>){+32767, +32767, +32767};
  _device = device_auto;
  io_timeout = 0;  ///< 0 = no timeout
  did_timeout = false;
  DeviceOk = false;
}

void MagneticSensorLsm303::EnableCalibration(bool _CalibrateMode)
{
    cal_running_min = { 32767, 32767, 32767 };
    cal_running_max = {-32768,-32768,-32768 };
    CalibrateMode = _CalibrateMode;
}

void MagneticSensorLsm303::UpdateCalibration(void)
{
    char report[120];

    cal_running_min.x = min(cal_running_min.x, magnetometer.x);
    cal_running_min.y = min(cal_running_min.y, magnetometer.y);
    cal_running_min.z = min(cal_running_min.z, magnetometer.z);

    cal_running_max.x = max(cal_running_max.x, magnetometer.x);
    cal_running_max.y = max(cal_running_max.y, magnetometer.y);
    cal_running_max.z = max(cal_running_max.z, magnetometer.z);

    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
        cal_running_min.x, cal_running_min.y, cal_running_min.z,
        cal_running_max.x, cal_running_max.y, cal_running_max.z);
    Serial.println(report);
}

///< Did a timeout occur in readAcc(), readMag(), or read() since the last call to timeoutOccurred()?
bool MagneticSensorLsm303::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void MagneticSensorLsm303::setTimeout(unsigned int timeout)
{
  io_timeout = timeout;
}

unsigned int MagneticSensorLsm303::getTimeout()
{
  return io_timeout;
}

bool MagneticSensorLsm303::init(deviceType device, sa0State sa0)
{
    Wire.begin();

  ///< perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
    ///< check for LSM303D if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_D)
    {
      ///< check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && myReadReg(D_SA0_HIGH_ADDRESS, WHO_AM_I) == D_WHO_ID)
      {
        ///< device responds to address 0011101 with D ID; it's a D with SA0 high
        device = device_D;
        sa0 = sa0_high;
      }
      ///< check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && myReadReg(D_SA0_LOW_ADDRESS, WHO_AM_I) == D_WHO_ID)
      {
        ///< device responds to address 0011110 with D ID; it's a D with SA0 low
        device = device_D;
        sa0 = sa0_low;
      }
    }
    
    ///< check for LSM303DLHC, DLM, DLH if device is still unidentified or was specified to be one of these types
    if (device == device_auto || device == device_DLHC || device == device_DLM || device == device_DLH)
    {
      ///< check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && myReadReg(DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS, CTRL_REG1_A) != TEST_REG_ERROR)
      {
        ///< device responds to address 0011001; it's a DLHC, DLM with SA0 high, or DLH with SA0 high
        sa0 = sa0_high;
        if (device == device_auto)
        { 
          ///< use magnetometer WHO_AM_I register to determine device type
          ///<
          ///< DLHC seems to respond to WHO_AM_I request the same way as DLM, even though this
          ///< register isn't documented in its datasheet. Since the DLHC accelerometer address is the
          ///< same as the DLM with SA0 high, but Pololu DLM boards pull SA0 low by default, we'll
          ///< guess that a device whose accelerometer responds to the SA0 high address and whose
          ///< magnetometer gives the DLM ID is actually a DLHC.
          device = (myReadReg(DLHC_DLM_DLH_MAG_ADDRESS, WHO_AM_I_M) == DLM_WHO_ID) ? device_DLHC : device_DLH;
        }
      }
      ///< check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && myReadReg(DLM_DLH_ACC_SA0_LOW_ADDRESS, CTRL_REG1_A) != TEST_REG_ERROR)
      {
        ///< device responds to address 0011000; it's a DLM with SA0 low or DLH with SA0 low
        sa0 = sa0_low;
        if (device == device_auto)
        {
          ///< use magnetometer WHO_AM_I register to determine device type
          device = (myReadReg(DLHC_DLM_DLH_MAG_ADDRESS, WHO_AM_I_M) == DLM_WHO_ID) ? device_DLM : device_DLH;
        }
      }
    }
    
    ///< make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }

  }
  DeviceOk = true;
  Serial.print("I2C Bus: "); Serial.println(I2CPort);
  Serial.print("Mag Addr: 0x"); Serial.println(DLHC_DLM_DLH_MAG_ADDRESS, HEX);
  Serial.print("Acc Addr: 0x"); Serial.println(DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS, HEX);

  Serial.print("Device:");

  
  _device = device;
  
  ///< set device addresses and translated register addresses
  switch (device)
  {
    case device_D:
      acc_address = mag_address = (sa0 == sa0_high) ? D_SA0_HIGH_ADDRESS : D_SA0_LOW_ADDRESS;
      translated_regs[-OUT_X_L_M] = D_OUT_X_L_M;
      translated_regs[-OUT_X_H_M] = D_OUT_X_H_M;
      translated_regs[-OUT_Y_L_M] = D_OUT_Y_L_M;
      translated_regs[-OUT_Y_H_M] = D_OUT_Y_H_M;
      translated_regs[-OUT_Z_L_M] = D_OUT_Z_L_M;
      translated_regs[-OUT_Z_H_M] = D_OUT_Z_H_M;
      Serial.println("LSM303D");
      break;

    case device_DLHC:
      acc_address = DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS; ///< DLHC doesn't have configurable SA0 but uses same acc address as DLM/DLH with SA0 high
      mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLHC_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLHC_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLHC_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLHC_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLHC_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLHC_OUT_Z_L_M;
      Serial.println("LSM303DLHC");
      break;

    case device_DLM:
      acc_address = (sa0 == sa0_high) ? DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS : DLM_DLH_ACC_SA0_LOW_ADDRESS;
      mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLM_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLM_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLM_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLM_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLM_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLM_OUT_Z_L_M;
      Serial.println("LSM303DLM");
      break;

    case device_DLH:
      acc_address = (sa0 == sa0_high) ? DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS : DLM_DLH_ACC_SA0_LOW_ADDRESS;
      mag_address = DLHC_DLM_DLH_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLH_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLH_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLH_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLH_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLH_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLH_OUT_Z_L_M;
      Serial.println("LSM303DLH");
      break;
  }
  
  return true;
}

void MagneticSensorLsm303::enable(void)
{

  if (_device == device_D)
  {
    ///< Accelerometer

    ///< 0x00 = 0b00000000
    ///< AFS = 0 (+/- 2 g full scale)
    writeReg(CTRL2, 0x00);

    ///< 0x57 = 0b01010111
    ///< AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
    writeReg(CTRL1, 0x57);

    ///< Magnetometer

    ///< 0x64 = 0b01100100
    ///< M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR)
    writeReg(CTRL5, 0x64);

    ///< 0x20 = 0b00100000
    ///< MFS = 01 (+/- 4 gauss full scale)
    writeReg(CTRL6, 0x20);

    ///< 0x00 = 0b00000000
    ///< MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
    writeReg(CTRL7, 0x00);

  }
  else
  {
    ///< Accelerometer
    
    if (_device == device_DLHC)
    {
      ///< 0x08 = 0b00001000
      ///< FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
        writeAccReg(CTRL_REG4_A, 0x08);

      ///< 0x47 = 0b01000111
      ///< ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
      writeAccReg(CTRL_REG1_A, 0x47);
    }
    else ///< DLM, DLH
    {
      ///< 0x00 = 0b00000000
      ///< FS = 00 (+/- 2 g full scale)
      writeAccReg(CTRL_REG4_A, 0x00);

      ///< 0x27 = 0b00100111
      ///< PM = 001 (normal mode); DR = 00 (50 Hz ODR); Zen = Yen = Xen = 1 (all axes enabled)
      writeAccReg(CTRL_REG1_A, 0x27);
    }

    ///< Magnetometer

    ///< 0x0C = 0b00001100
    ///< DO = 011 (7.5 Hz ODR)
    writeMagReg(CRA_REG_M, 0x8c); // was 0c now 8c to enable the Temperature Sensor in DLHC.

    ///< 0x20 = 0b00100000
    ///< GN = 001 (+/- 1.3 gauss full scale)
    writeMagReg(CRB_REG_M, 0x20);

    ///< 0x00 = 0b00000000
    ///< MD = 00 (continuous-conversion mode)
    writeMagReg(MR_REG_M, 0x00);

  }
}

///< Writes an accelerometer register
void MagneticSensorLsm303::writeAccReg(byte reg, byte value)
{
    Wire.beginTransmission(acc_address);
    Wire.write(reg);
    Wire.write(value);
  last_status = Wire.endTransmission();
}

///< Reads an accelerometer register
byte MagneticSensorLsm303::readAccReg(byte reg)
{
  byte value;
  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}

///< Writes a magnetometer register
void MagneticSensorLsm303::writeMagReg(byte reg, byte value)
{
    Wire.beginTransmission(mag_address);
    Wire.write(reg);
    Wire.write(value);
  last_status = Wire.endTransmission();
}

///< Reads a magnetometer register
byte MagneticSensorLsm303::readMagReg(int reg)
{
  byte value;

  ///< if dummy register address (magnetometer Y/Z), look up actual translated address (based on device type)
  if (reg < 0)
  {
    reg = translated_regs[-reg];
  }

  Wire.beginTransmission(mag_address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(mag_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void MagneticSensorLsm303::writeReg(byte reg, byte value)
{
  ///< mag address == acc_address for LSM303D, so it doesn't really matter which one we use.
  if (_device == device_D || reg < CTRL_REG1_A)
  {
    writeMagReg(reg, value);
  }
  else
  {
    writeAccReg(reg, value);
  }
}

///< Note that this function will not work for reading TEMP_OUT_H_M and TEMP_OUT_L_M on the DLHC.
///< To read those two registers, use readMagReg() instead.
byte MagneticSensorLsm303::readReg(int reg)
{
  ///< mag address == acc_address for LSM303D, so it doesn't really matter which one we use.
  ///< Use readMagReg so it can translate OUT_[XYZ]_[HL]_M
  if (_device == device_D || reg < CTRL_REG1_A)
  {
    return readMagReg(reg);
  }
  else
  {
    return readAccReg(reg);
  }
}

///< Reads the 3 accelerometer channels and stores them in vector a
void MagneticSensorLsm303::readAcc(void)
{
    Wire.beginTransmission(acc_address);
  ///< assert the MSB of the address to get the accelerometer
  ///< to do slave-transmit subaddress updating.
    Wire.write(OUT_X_L_A | (1 << 7));
  last_status = Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6)
  {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  ///< combine high and low bytes
  ///< This no longer drops the lowest 4 bits of the readings from the DLH/DLM/DLHC, which are always 0
  ///< (12-bit resolution, left-aligned). The D has 16-bit resolution
  accelerometer.x = (int16_t)(xha << 8 | xla);
  accelerometer.y = (int16_t)(yha << 8 | yla);
  accelerometer.z = (int16_t)(zha << 8 | zla);
}

///< Reads the 3 magnetometer channels and stores them in vector m
void MagneticSensorLsm303::readMag(void)
{
    Wire.beginTransmission(mag_address);
  ///< If LSM303D, assert MSB to enable subaddress updating
  ///< OUT_X_L_M comes first on D, OUT_X_H_M on others
    Wire.write((_device == device_D) ? translated_regs[-OUT_X_L_M] | (1 << 7) : translated_regs[-OUT_X_H_M]);
  last_status = Wire.endTransmission();
  Wire.requestFrom(mag_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6)
  {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  byte xlm, xhm, ylm, yhm, zlm, zhm;

  if (_device == device_D)
  {
    ///< D: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    xlm = Wire.read();
    xhm = Wire.read();
    ylm = Wire.read();
    yhm = Wire.read();
    zlm = Wire.read();
    zhm = Wire.read();
  }
  else
  {
    ///< DLHC, DLM, DLH: X_H, X_L...
    xhm = Wire.read();
    xlm = Wire.read();

    if (_device == device_DLH)
    {
      ///< DLH: ...Y_H, Y_L, Z_H, Z_L
      yhm = Wire.read();
      ylm = Wire.read();
      zhm = Wire.read();
      zlm = Wire.read();
    }
    else
    {
      ///< DLM, DLHC: ...Z_H, Z_L, Y_H, Y_L
      zhm = Wire.read();
      zlm = Wire.read();
      yhm = Wire.read();
      ylm = Wire.read();
    }
  }

  ///< combine high and low bytes
  magnetometer.x = (int16_t)(xhm << 8 | xlm);
  magnetometer.y = (int16_t)(yhm << 8 | ylm);
  magnetometer.z = (int16_t)(zhm << 8 | zlm);
}

///< Reads all 6 channels of the MagneticSensorLsm303 and stores them in the object variables
void MagneticSensorLsm303::read(void)
{
  readAcc();
  readMag();

  pitch = (atan2(-accelerometer.x, sqrt((long)accelerometer.y * accelerometer.y + (long)accelerometer.z * accelerometer.z)) * 180) / PI;
  roll = (atan2(accelerometer.y, accelerometer.z) * 180) / PI;

  if (CalibrateMode)
      UpdateCalibration();
}

float MagneticSensorLsm303::getNavigationAngle(void)
{
  if (_device == device_D)
  {
    return heading((vector<int>){1, 0, 0});
  }
  else
  {
    return heading((vector<int>){0, -1, 0});
  }
}

void MagneticSensorLsm303::vectorNormalize(vector<float> *a)
{
  float mag = sqrt(vectorDot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

template <typename T> float MagneticSensorLsm303::heading(vector<T> from)
{
    vector<int32_t> temp_m = {magnetometer.x, magnetometer.y, magnetometer.z};

    ///< subtract offset (average of min and max) from magnetometer readings
    temp_m.x -= ((int32_t)magnetometer_min.x + magnetometer_max.x) / 2;
    temp_m.y -= ((int32_t)magnetometer_min.y + magnetometer_max.y) / 2;
    temp_m.z -= ((int32_t)magnetometer_min.z + magnetometer_max.z) / 2;

    ///< compute E and N
    vector<float> E;
    vector<float> N;
    vectorCross(&temp_m, &accelerometer, &E);
    vectorNormalize(&E);
    vectorCross(&accelerometer, &E, &N);
    vectorNormalize(&N);

    ///< compute heading
    float heading = atan2(vectorDot(&E, &from), vectorDot(&N, &from)) * 180 / M_PI;
    if (heading < 0) heading += 360;
    return heading;
}

template <typename Ta, typename Tb, typename To> void MagneticSensorLsm303::vectorCross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float MagneticSensorLsm303::vectorDot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

int MagneticSensorLsm303::myReadReg(byte address, regAddr reg)
{
    Wire.beginTransmission(address);
    Wire.write((byte)reg);
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, (byte)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}

float MagneticSensorLsm303::readTempC()
{
  //  uint8_t valueL;
  //  uint8_t valueH;
    float temperature;

    valueH = readMagReg(TEMP_OUT_H_M);
    valueL = readMagReg(TEMP_OUT_L_M);

    temperature = (float)((valueH << 4) + (valueL >> 4) );
    temperature /= 8; // 8 digits/?C
    temperature += 20;// Reads 0 @ 25?C

    return temperature;
}

