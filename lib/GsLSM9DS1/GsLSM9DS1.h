#ifndef __GsLSM9DS1_H__
#define __GsLSM9DS1_H__

#include <mbed.h>
#include "GsLSM9DS1_Constants.h"
#include "LSM9DS1_Registers.h"

#define AG_ADDRESS(sa0) ((sa0) ? 0xD6 : 0xD4) // acceleration & gyro
#define M_ADDRESS(sa1)  ((sa1) ? 0x3C : 0x38) // magnetic sensor

struct gyroSettings
{
    // Gyroscope settings:
    uint8_t enabled;
    uint16_t scale; // Changed this to 16-bit
    uint8_t sampleRate;
    // New gyro stuff:
    uint8_t bandwidth;
    uint8_t lowPowerEnable;
    uint8_t HPFEnable;
    uint8_t HPFCutoff;
    uint8_t flipX;
    uint8_t flipY;
    uint8_t flipZ;
    uint8_t orientation;
    uint8_t enableX;
    uint8_t enableY;
    uint8_t enableZ;
    uint8_t latchInterrupt;
};

struct deviceSettings
{
    uint8_t commInterface; // Can be I2C, SPI 4-wire or SPI 3-wire
    uint8_t agAddress;  // I2C address or SPI CS pin
    uint8_t mAddress;   // I2C address or SPI CS pin
};

struct accelSettings
{
    // Accelerometer settings:
    uint8_t enabled;
    accel_scale scale;
    uint8_t sampleRate;
    // New accel stuff:
    uint8_t enableX;
    uint8_t enableY;
    uint8_t enableZ;
    int8_t  bandwidth;
    uint8_t highResEnable;
    uint8_t highResBandwidth;
};

struct magSettings
{
    // Magnetometer settings:
    uint8_t enabled;
    uint8_t scale;
    uint8_t sampleRate;
    // New mag stuff:
    uint8_t tempCompensationEnable;
    uint8_t XYPerformance;
    uint8_t ZPerformance;
    uint8_t lowPowerEnable;
    uint8_t operatingMode;
};

struct temperatureSettings
{
    // Temperature settings
    uint8_t enabled;
};

struct IMUSettings
{
    deviceSettings device;
    
    gyroSettings gyro;
    accelSettings accel;
    magSettings mag;
    
    temperatureSettings temp;
};


class GsLSM9DS1 {
public:

    IMUSettings settings;

    // We'll store the gyro, accel, and magnetometer readings in a series of
    // public class variables. Each sensor gets three variables -- one for each
    // axis. Call readGyro(), readAccel(), and readMag() first, before using
    // these variables!
    // These values are the RAW signed 16-bit readings from the sensors.
    int16_t gx_raw, gy_raw, gz_raw; // x, y, and z axis readings of the gyroscope
    int16_t ax_raw, ay_raw, az_raw; // x, y, and z axis readings of the accelerometer
    int16_t mx_raw, my_raw, mz_raw; // x, y, and z axis readings of the magnetometer
    int16_t temperature_raw;
    
    // 静止状態での値のゆらぎをバイアスとして格納する
    int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];    // サンプリング合計値をサンプリング数で割った値
    float gBias[3], aBias[3], mBias[3];             // 上記値に単位値をかけ合わせた値 MEMO:計算してるけど使ってない


    // floating-point values of scaled data in real-world units
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    float temperature_c; // temperature
        
    // Constructor
    GsLSM9DS1(PinName sda, PinName scl, uint8_t agAddr = AG_ADDRESS(1), uint8_t mAddr = M_ADDRESS(1));
    
    /**  init() -- Initialize the gyro, accelerometer, and magnetometer.
    *  This will set up the scale and output rate of each sensor. It'll also
    *  "turn on" every sensor and every axis of every sensor.
    *  Input:
    *   - gScl = The scale of the gyroscope. This should be a gyro_scale value.
    *   - aScl = The scale of the accelerometer. Should be a accel_scale value.
    *   - mScl = The scale of the magnetometer. Should be a mag_scale value.
    *   - gODR = Output data rate of the gyroscope. gyro_odr value.
    *   - aODR = Output data rate of the accelerometer. accel_odr value.
    *   - mODR = Output data rate of the magnetometer. mag_odr value.
    *  Output: The function will return an unsigned 16-bit value. The most-sig
    *       bytes of the output are the WHO_AM_I reading of the accel/gyro. The
    *       least significant two bytes are the WHO_AM_I reading of the mag.
    *  All parameters have a defaulted value, so you can call just "begin()".
    *  Default values are FSR's of: +/- 245DPS, 4g, 2Gs; ODRs of 119 Hz for 
    *  gyro, 119 Hz for accelerometer, 80 Hz for magnetometer.
    *  Use the return value of this function to verify communication.
    */
    void init(accel_scale aScl = A_SCALE_2G,
                  gyro_scale  gScl = G_SCALE_245DPS,
                  mag_scale   mScl = M_SCALE_4GS,
                  accel_odr   aODR = A_ODR_119,
                  gyro_odr    gODR = G_ODR_119_BW_14,
                  mag_odr     mODR = M_ODR_80);

    /** begin() -- Initialize the gyro, accelerometer, and magnetometer.
    *This will set up the scale and output rate of each sensor. The values set
    * in the IMUSettings struct will take effect after calling this function.
    */
    uint16_t begin();

    // Check -------------------------------------------------------------------

    // Perform who_am_i test for A/G/M
    uint16_t checkWhoAmI();
    
        /** accelAvailable() -- Polls the accelerometer status register to check
    * if new data is available.
    * Output:  1 - New data available
    *          0 - No new data available
    */
    uint8_t accelAvailable();
    
    /** gyroAvailable() -- Polls the gyroscope status register to check
    * if new data is available.
    * Output:  1 - New data available
    *          0 - No new data available
    */
    uint8_t gyroAvailable();
    
    /** gyroAvailable() -- Polls the temperature status register to check
    * if new data is available.
    * Output:  1 - New data available
    *          0 - No new data available
    */
    uint8_t tempAvailable();
    
    /** magAvailable() -- Polls the accelerometer status register to check
    * if new data is available.
    * Input:
    *  - axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
    *    on one specific axis. Or ALL_AXIS (default) to check for new data
    *    on all axes.
    * Output:  1 - New data available
    *          0 - No new data available
    */
    uint8_t magAvailable(lsm9ds1_axis axis = ALL_AXIS);

    // Calibration ------------------------------------------------------------

    void calibrate(bool autoCalc = true);
    void calibrateMag(bool loadIn = true);
    void magOffset(uint8_t axis, int16_t offset);

    // FIFO  ------------------------------------------------------------------

    /** enableFIFO() - Enable or disable the FIFO
    * Input:
    *  - enable: true = enable, false = disable.
    */
    void enableFIFO(bool enable = true);
    
    /** setFIFO() - Configure FIFO mode and Threshold
    * Input:
    *  - fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
    *    Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
    *  - fifoThs: FIFO threshold level setting
    *    Any value from 0-0x1F is acceptable.
    */
    void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);
    
    //! getFIFOSamples() - Get number of FIFO samples
    uint8_t getFIFOSamples();

    // Read  ------------------------------------------------------------------

    // xmReadByte() -- Read a byte from a register in the accel/mag sensor
    // Input:
    //  - subAddress = Register to be read from.
    // Output:
    //  - An 8-bit value read from the requested register.
    uint8_t agReadByte(uint8_t subAddress);
    
    // xmReadBytes() -- Reads a number of bytes -- beginning at an address
    // and incrementing from there -- from the accelerometer/magnetometer.
    // Input:
    //  - subAddress = Register to be read from.
    //  - * dest = A pointer to an array of uint8_t's. Values read will be
    //      stored in here on return.
    //  - count = The number of bytes to be read.
    // Output: No value is returned, but the `dest` array will store
    //  the data read upon exit.
    void agReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

    // gReadByte() -- Reads a byte from a specified gyroscope register.
    // Input:
    //  - subAddress = Register to be read from.
    // Output:
    //  - An 8-bit value read from the requested address.
    uint8_t mReadByte(uint8_t subAddress);

    // readGyro() -- Read the gyroscope output registers.
    // This function will read all six gyroscope output registers.
    // The readings are stored in the class' gx, gy, and gz variables. Read
    // those _after_ calling readGyro().
    void readGyro();
        
    // readAccel() -- Read the accelerometer output registers.
    // This function will read all six accelerometer output registers.
    // The readings are stored in the class' ax, ay, and az variables. Read
    // those _after_ calling readAccel().
    void readAccel();
        
    // readMag() -- Read the magnetometer output registers.
    // This function will read all six magnetometer output registers.
    // The readings are stored in the class' mx, my, and mz variables. Read
    // those _after_ calling readMag().
    void readMag();
     
    // readTemp() -- Read the temperature output register.
    // This function will read two temperature output registers.
    // The combined readings are stored in the class' temperature variables. Read
    // those _after_ calling readTemp().
    void readTemp();

    // Write ------------------------------------------------------------------

    // agWriteByte() -- Write a byte to a register in the accel/mag sensor.
    // Input:
    //  - subAddress = Register to be written to.
    //  - data = data to be written to the register.
    void agWriteByte(uint8_t subAddress, uint8_t data);

    // mWriteByte() -- Write a byte to a register in the magnetorometor.
    // Input:
    //  - subAddress = Register to be written to.
    //  - data = data to be written to the register.
    void mWriteByte(uint8_t subAddress, uint8_t data);

protected:

    uint8_t agAddress, mAddress;
    char cmd[2];
    
    /**  gScale, aScale, and mScale store the current scale range for each 
    *  sensor. Should be updated whenever that value changes.
    */
    gyro_scale  gScale;
    accel_scale aScale;
    mag_scale   mScale;
    
    /**  gRes, aRes, and mRes store the current resolution for each sensor. 
    *  Units of these values would be DPS (or g's or Gs's) per ADC tick.
    *  This value is calculated as (sensor scale) / (2^15).
    */
    float gRes, aRes, mRes;

    // _autoCalc keeps track of whether we're automatically subtracting off
    // accelerometer and gyroscope bias calculated in calibrate().
    uint8_t _autoCalc;

// I2C  -----------------------------------------------------------------------
    
    uint8_t i2cRead(int address, char subAddress);
    
    // I2CreadByte() -- Read a single byte from a register over I2C.
    // Input:
    //  - address = The 7-bit I2C address of the slave device.
    //  - subAddress = The register to be read from.
    // Output:
    //  - The byte read from the requested address.
    uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);

    // I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
    // Input:
    //  - address = The 7-bit I2C address of the slave device.
    //  - subAddress = The register to begin reading.
    //  - * dest = Pointer to an array where we'll store the readings.
    //  - count = Number of registers to be read.
    // Output: No value is returned by the function, but the registers read are
    //      all stored in the *dest array given.
    uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);

    // I2CwriteByte() -- Write a byte out of I2C to a register in the device
    // Input:
    //  - address = The 7-bit I2C address of the slave device.
    //  - subAddress = The register to be written to.
    //  - data = Byte to be written to the register.
    void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);

// Calculation  ---------------------------------------------------------------

    void calcaRes();
    void calcgRes();
    void calcmRes();

    /** calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
    * This function reads in a signed 16-bit value and returns the scaled
    * DPS. This function relies on gScale and gRes being correct.
    * Input:
    *  - gyro = A signed 16-bit raw reading from the gyroscope.
    */
    float calcGyro(int16_t gyro);
    
    /** calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
    * This function reads in a signed 16-bit value and returns the scaled
    * g's. This function relies on aScale and aRes being correct.
    * Input:
    *  - accel = A signed 16-bit raw reading from the accelerometer.
    */
    float calcAccel(int16_t accel);
    
    /** calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
    * This function reads in a signed 16-bit value and returns the scaled
    * Gs. This function relies on mScale and mRes being correct.
    * Input:
    *  - mag = A signed 16-bit raw reading from the magnetometer.
    */
    float calcMag(int16_t mag);
    
// Init  ----------------------------------------------------------------------

    /**  initAccel() -- Sets up the accelerometer to begin reading.
    *  This function steps through all accelerometer related control registers.
    */
    void initAccel();
    
    /**  initGyro() -- Sets up the gyroscope to begin reading.
    *  This function steps through all three gyroscope control registers.
    */
    void initGyro();

    /**  initMag() -- Sets up the magnetometer to begin reading.
    *  This function steps through all magnetometer-related control registers.
    */
    void initMag();

// Set ODR  -------------------------------------------------------------------

    /**  setAccelODR() -- Set the output data rate of the accelerometer
    *  Input:
    *   - aODR = The desired output rate of the accel.
    */
    void setAccelODR(accel_odr aODR); // Set the accel output data rate.

    /**  setAccelScale() -- Set the full-scale range of the accelerometer.
    *  This function can be called to set the scale of the accelerometer to
    *  2, 4, 8, or 16 g's.
    *  Input:
    *   - aScl = The desired accelerometer scale. Must be one of five possible
    *       values from the accel_scale enum.
    */
    void setAccelScale(accel_scale aScl);

    /**  setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
    *  Input:
    *   - gRate = The desired output rate and cutoff frequency of the gyro.
    *       Must be a value from the gyro_odr enum (check above).
    */
    void setGyroODR(gyro_odr gODR);

    /**  setMagODR() -- Set the output data rate of the magnetometer
    *  Input:
    *   - mODR = The desired output rate of the mag.
    *       Must be a value from the mag_odr enum (check above).
    */
    void setMagODR(mag_odr mODR);
        
// Set Scale  -----------------------------------------------------------------

    /**  setGyroScale() -- Set the full-scale range of the gyroscope.
    *  This function can be called to set the scale of the gyroscope to 
    *  245, 500, or 2000 degrees per second.
    *  Input:
    *   - gODR = The desired gyroscope scale. Must be one of three possible
    *       values from the gyro_scale enum.
    */
    void setGyroScale(gyro_scale gScl);

    /**  setMagScale() -- Set the full-scale range of the magnetometer.
    *  This function can be called to set the scale of the magnetometer to
    *  4, 8, 12, or 16 Gs.
    *  Input:
    *   - mScl = The desired magnetometer scale. Must be one of four possible
    *       values from the mag_scale enum.
    */
    void setMagScale(mag_scale mScl);

private:
    I2C i2c;
};

#endif