/*!
 * @file MagenticSensorLsm303.h
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


/* LSM303DLH Example Code
 
   LSM303 Breakout ---------- Arduino
         Vin                   5V
         GND                   GND
         SDA                   A4
         SCL                   A5
*/
#ifndef MAGNETICSENSORLSM303_h
#define MAGNETICSENSORLSM303_h
#include <Arduino.h> ///< for byte data type
///< The Arduino two-wire interface uses a 7-bit number for the address,
///< and sets the last bit correctly based on reads and writes
#define D_SA0_HIGH_ADDRESS                0b0011101 //x1D
#define D_SA0_LOW_ADDRESS                 0b0011110
#define DLHC_DLM_DLH_MAG_ADDRESS          0b0011110 //x1E
#define DLHC_DLM_DLH_ACC_SA0_HIGH_ADDRESS 0b0011001
#define DLM_DLH_ACC_SA0_LOW_ADDRESS       0b0011000
#define TEST_REG_ERROR -1
#define D_WHO_ID    0x49
#define DLM_WHO_ID  0x3C


class MagneticSensorLsm303
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };
    enum deviceType 
	{ 
		device_DLH, 
		device_DLM, 
		device_DLHC, 
		device_D, 
		device_auto 
	};
    enum sa0State 
	{ 
		sa0_low, 
		sa0_high, 
		sa0_auto 
	};
    ///< register addresses
    enum regAddr
    {
      TEMP_OUT_L_LSM303        = 0x05, ///< D   // disambiguated
      TEMP_OUT_H_LSM303        = 0x06, ///< D   // disambiguated

      STATUS_M          = 0x07, ///< D

      INT_CTRL_M        = 0x12, ///< D
      INT_SRC_M         = 0x13, ///< D
      INT_THS_L_M       = 0x14, ///< D
      INT_THS_H_M       = 0x15, ///< D

      OFFSET_X_L_M      = 0x16, ///< D
      OFFSET_X_H_M      = 0x17, ///< D
      OFFSET_Y_L_M      = 0x18, ///< D
      OFFSET_Y_H_M      = 0x19, ///< D
      OFFSET_Z_L_M      = 0x1A, ///< D
      OFFSET_Z_H_M      = 0x1B, ///< D
      REFERENCE_X       = 0x1C, ///< D
      REFERENCE_Y       = 0x1D, ///< D
      REFERENCE_Z       = 0x1E, ///< D

      CTRL0             = 0x1F, ///< D
      CTRL1             = 0x20, ///< D
      CTRL_REG1_A       = 0x20, ///< DLH, DLM, DLHC
      CTRL2             = 0x21, ///< D
      CTRL_REG2_A       = 0x21, ///< DLH, DLM, DLHC
      CTRL3             = 0x22, ///< D
      CTRL_REG3_A       = 0x22, ///< DLH, DLM, DLHC
      CTRL4             = 0x23, ///< D
      CTRL_REG4_A       = 0x23, ///< DLH, DLM, DLHC
      CTRL5             = 0x24, ///< D
      CTRL_REG5_A       = 0x24, ///< DLH, DLM, DLHC
      CTRL6             = 0x25, ///< D
      CTRL_REG6_A       = 0x25, ///< DLHC
      HP_FILTER_RESET_A = 0x25, ///< DLH, DLM
      CTRL7             = 0x26, ///< D
      REFERENCE_A       = 0x26, ///< DLH, DLM, DLHC
      STATUS_A          = 0x27, ///< D
      STATUS_REG_A      = 0x27, ///< DLH, DLM, DLHC

      OUT_X_L_A         = 0x28,
      OUT_X_H_A         = 0x29,
      OUT_Y_L_A         = 0x2A,
      OUT_Y_H_A         = 0x2B,
      OUT_Z_L_A         = 0x2C,
      OUT_Z_H_A         = 0x2D,

      FIFO_CTRL         = 0x2E, ///< D
      FIFO_CTRL_REG_A   = 0x2E, ///< DLHC
      FIFO_SRC          = 0x2F, ///< D
      FIFO_SRC_REG_A    = 0x2F, ///< DLHC

      TEMP_OUT_H_M = 0x31, // DLHC
      TEMP_OUT_L_M = 0x32, // DLHC

      IG_CFG1           = 0x30, ///< D
      INT1_CFG_A        = 0x30, ///< DLH, DLM, DLHC
      IG_SRC1           = 0x31, ///< D
      INT1_SRC_A        = 0x31, ///< DLH, DLM, DLHC
      IG_THS1           = 0x32, ///< D
      INT1_THS_A        = 0x32, ///< DLH, DLM, DLHC
      IG_DUR1           = 0x33, ///< D
      INT1_DURATION_A   = 0x33, ///< DLH, DLM, DLHC
      IG_CFG2           = 0x34, ///< D
      INT2_CFG_A        = 0x34, ///< DLH, DLM, DLHC
      IG_SRC2           = 0x35, ///< D
      INT2_SRC_A        = 0x35, ///< DLH, DLM, DLHC
      IG_THS2           = 0x36, ///< D
      INT2_THS_A        = 0x36, ///< DLH, DLM, DLHC
      IG_DUR2           = 0x37, ///< D
      INT2_DURATION_A   = 0x37, ///< DLH, DLM, DLHC

      CLICK_CFG         = 0x38, ///< D
      CLICK_CFG_A       = 0x38, ///< DLHC
      CLICK_SRC         = 0x39, ///< D
      CLICK_SRC_A       = 0x39, ///< DLHC
      CLICK_THS         = 0x3A, ///< D
      CLICK_THS_A       = 0x3A, ///< DLHC
      TIME_LIMIT        = 0x3B, ///< D
      TIME_LIMIT_A      = 0x3B, ///< DLHC
      TIME_LATENCY      = 0x3C, ///< D
      TIME_LATENCY_A    = 0x3C, ///< DLHC
      TIME_WINDOW       = 0x3D, ///< D
      TIME_WINDOW_A     = 0x3D, ///< DLHC

      Act_THS           = 0x3E, ///< D
      Act_DUR           = 0x3F, ///< D

      CRA_REG_M         = 0x00, ///< DLH, DLM, DLHC
      CRB_REG_M         = 0x01, ///< DLH, DLM, DLHC
      MR_REG_M          = 0x02, ///< DLH, DLM, DLHC

      SR_REG_M          = 0x09, ///< DLH, DLM, DLHC
      IRA_REG_M         = 0x0A, ///< DLH, DLM, DLHC
      IRB_REG_M         = 0x0B, ///< DLH, DLM, DLHC
      IRC_REG_M         = 0x0C, ///< DLH, DLM, DLHC

      WHO_AM_I          = 0x0F, ///< D
      WHO_AM_I_M        = 0x0F, ///< DLM


      ///< dummy addresses for registers in different locations on different devices;
      ///< the library translates these based on device type
      ///< value with sign flipped is used as index into translated_regs array

      OUT_X_H_M         = -1,
      OUT_X_L_M         = -2,
      OUT_Y_H_M         = -3,
      OUT_Y_L_M         = -4,
      OUT_Z_H_M         = -5,
      OUT_Z_L_M         = -6,
      ///< update dummy_reg_count if registers are added here!

      ///< device-specific register addresses

      DLH_OUT_X_H_M     = 0x03,
      DLH_OUT_X_L_M     = 0x04,
      DLH_OUT_Y_H_M     = 0x05,
      DLH_OUT_Y_L_M     = 0x06,
      DLH_OUT_Z_H_M     = 0x07,
      DLH_OUT_Z_L_M     = 0x08,

      DLM_OUT_X_H_M     = 0x03,
      DLM_OUT_X_L_M     = 0x04,
      DLM_OUT_Z_H_M     = 0x05,
      DLM_OUT_Z_L_M     = 0x06,
      DLM_OUT_Y_H_M     = 0x07,
      DLM_OUT_Y_L_M     = 0x08,

      DLHC_OUT_X_H_M    = 0x03,
      DLHC_OUT_X_L_M    = 0x04,
      DLHC_OUT_Z_H_M    = 0x05,
      DLHC_OUT_Z_L_M    = 0x06,
      DLHC_OUT_Y_H_M    = 0x07,
      DLHC_OUT_Y_L_M    = 0x08,

      D_OUT_X_L_M       = 0x08,
      D_OUT_X_H_M       = 0x09,
      D_OUT_Y_L_M       = 0x0A,
      D_OUT_Y_H_M       = 0x0B,
      D_OUT_Z_L_M       = 0x0C,
      D_OUT_Z_H_M       = 0x0D
    };

    vector<int16_t> accelerometer; ///< accelerometer readings
    vector<int16_t> magnetometer; ///< magnetometer readings
    vector<int16_t> magnetometer_max; ///< maximum magnetometer values, used for calibration
    vector<int16_t> magnetometer_min; ///< minimum magnetometer values, used for calibration
    bool CalibrateMode;
    void EnableCalibration(bool _CalibrateMode);
    void UpdateCalibration(void);
    vector<int16_t> cal_running_min; 
    vector<int16_t> cal_running_max; 

	/*!
	*  @brief Constructor
	*/
    MagneticSensorLsm303(void);
	
	/*!
   *  @brief initializes the library with the device being used (device_DLH, device_DLM,
   *         device_DLHC, device_D, or device_auto) and the state of the SA0 pin (sa0_low, 
   *         sa0_high, or sa0_auto), which determines the least significant bit(s) of the 
   *         I²C slave address (on some devices, and only for the accelerometer in some cases). 
   *         Constants for these arguments are defined in LSM303.h. Both of these arguments are 
   *         optional; if they are not specified, the library will try to automatically detect 
   *         the device and accelerometer address1. A boolean is returned indicating whether the 
   *         type of LSM303 device was successfully determined (if necessary)
   *
   *  @param device  deviceType.
   *  @Param sa0_auto sa0State
   *  @return 0 success
   */
    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
	
	/*!
	*  @brief Turns on the accelerometer and magnetometer and enables a consistent set of default settings.
	*         This function will set the accelerometer’s full scale to be +/-2 g, which means that a reading 
	*         of 16384 corresponds to approximately 1 g. The magnetometer’s full scale is set to +/-4 gauss
	*	      for the LSM303D or +/-1.3 gauss on all other devices. See the comments in LSM303.cpp for a 
	*         full explanation of these settings.
	*/
    void enable(void);

	/*!
	*	@brief Takes a reading from both the accelerometer and magnetometer and stores the values in the vectors a and m
	*/
    void read(void);
	
	/*!
	*  @brief Sets a timeout period for readAcc() and readMag(), in milliseconds, 
	*         after which they will abort if no data is received. A value of 0 disables the timeout
	*/
    void setTimeout(unsigned int timeout);
	
	/*!
	*	@brief Get the current timeout period setting
	*   @return the current timeout period setting
	*/
    unsigned int getTimeout(void);
	
	/*!
	*	@brief Get a boolean indicating whether a call to readAcc() or readMag() has timed 
	*          out since the last call to timeoutOccurred()
	*
	*   @return boolean true/false
	*/
    bool timeoutOccurred(void);
	
	/*!
	*	@brief Get the tilt-compensated heading of a default vector in degrees 
	*         (the angular difference in the horizontal plane between the default vector and north).
    *		   The default vector is chosen to point along the surface of the PCB, in the direction 
	*          of the top of the text on the silkscreen. This is the +X axis on the Pololu LSM303D 
	*          carrier and the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
	*
	*	@return the value of drift angle.
	*/
    float getNavigationAngle(void);

    float pitch;
    float roll;
    float readTempC();
    uint8_t I2CPort;
    boolean DeviceOk;

  private:
    uint8_t valueL;
    uint8_t valueH;
    deviceType _device; ///< chip type (D, DLHC, DLM, or DLH)
    byte acc_address; ///<accelerometer regedit address
    byte mag_address; ///<magnetometer regedit address
    static const int dummy_reg_count = 6;
    regAddr translated_regs[dummy_reg_count + 1]; ///< index 0 not used
    unsigned int io_timeout; ///<timeout
    bool did_timeout;///< status of timeout
	byte last_status; ///< status of last I2C transmission
	
	/*!
   *  @brief Get Read the value of the register
   *
   *  @param address  accelerometer Register address
   *  @param reg  accelerometer Register data
   *  @return value of the register
   */
    int myReadReg(byte address, regAddr reg);
	
	/*!
	*	@brief Get the tilt-compensated heading of the given vector in degrees 
	*          (the angular difference in the horizontal plane between from and north).
	*
	*	@Returns the angular difference in the horizontal plane between the
	*    		"from" vector and north, in degrees
	*
	*/
    template <typename T> float heading(vector<T> from);

	/*!
	*	@brief  The number of three sum of squares
	*
	*	@param a input data
	*	@param b input data
	*/
    template <typename Ta, typename Tb, typename To> static void vectorCross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
	
	/*!
	*	@brief Sum of squares
	*
	*	@param a input data
	* 	@param b input data
	*	return The calculation results
	*/
    template <typename Ta, typename Tb> static float vectorDot(const vector<Ta> *a, const vector<Tb> *b);
	
	/*!
	*	@brief get the standard results 
	*
	*	@param a input data
	*/
    static void vectorNormalize(vector<float> *a);
	
   /*!
   *  @brief Writes an accelerometer register with the given value
   *
   *  @param value  accelerometer Register data
   *  @param reg  accelerometer Register address
   */
    void writeAccReg(byte reg, byte value);
	
	/*!
   *  @brief Reads an accelerometer register and returns the value read.
   *
   *  @param reg accelerometer Register address,
   *  @return the value of the accelerometer register to the address.
   */
    byte readAccReg(byte reg);
	
	/*!
   *  @brief Writes a magnetometer register with the given value..
   *
   *  @param value  magnetometer Register data
   *  @param reg  magnetometer Register address
   */
    void writeMagReg(byte reg, byte value);
	
	/*!
   *  @brief Reads a magnetometer register and returns the value read.
   *
   *  @param reg magnetometer Register address,
   *  @return the value of the magnetometer register to the address.
   */
    byte readMagReg(int reg);
	
	/*!
   *  @brief Writes an accelerometer or magnetometer register with the given value. 
   *        Register addresses are defined by the regAddr enumeration type in LSM303.h.
   *
   *  @param value  Register data
   *  @param reg  Register address
   */
    void writeReg(byte reg, byte value);
	
	/*!
   *  @brief Reads an accelerometer or magnetometer register and returns the value read.
   *
   *  @param reg Register address,
   *  @return the value of the register to the address.
   */
    byte readReg(int reg);
	
	/*!
	* @brief Takes a reading from the accelerometer and stores the values in the vector a. 
	*        Conversion of the readings to units of g depends on the accelerometer’s selected 
	*        gain (full scale setting).Note that in the LSM303DLHC, LSM303DLM, and LSM303DLH, 
	*        the acceleration data registers actually contain a left-aligned 12-bit number, 
	*        so the lowest 4 bits are always 0, and the values in a should be shifted right 
	*        by 4 bits (divided by 16) to be consistent with the conversion factors specified 
	*        in the datasheets
	*/
    void readAcc(void);
	
	/*! 
	*  @brief Takes a reading from the magnetometer and stores the values in the vector m. Conversion
    *	      of the readings to units of gauss depends on the magnetometer’s selected gain (full scale setting).
	*/
    void readMag(void);
};



#endif



