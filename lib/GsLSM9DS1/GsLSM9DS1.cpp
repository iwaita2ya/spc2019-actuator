#ifndef __GsLSM9DS1_CPP__
#define __GsLSM9DS1_CPP__

#include "GsLSM9DS1.h"
#include "GsLSM9DS1_Constants.h"

// Constructor ----------------------------------------------------------------
GsLSM9DS1::GsLSM9DS1(PinName sda, PinName scl, uint8_t _agAddress, uint8_t _mAddress) : i2c(sda, scl)
{
    // store 7-bit I2C address
    agAddress = _agAddress;
    mAddress  = _mAddress;
}

// INIT -----------------------------------------------------------------------
/**
 * 各センサーの初期設定を行う。データ取得開始は begin() をコール
 * 　センサー有効／無効
 * 　スケール
 * 　サンプリングレート
 * 　カットオフ周波数
 * @param aScl
 * @param gScl
 * @param mScl
 * @param aODR
 * @param gODR
 * @param mODR
 * @return
 */
void GsLSM9DS1::init(
    accel_scale aScl,
    gyro_scale  gScl,
    mag_scale   mScl, 
    accel_odr   aODR,
    gyro_odr    gODR,
    mag_odr     mODR)
{

    //-----------------------
    // ジャイロ設定
    //-----------------------
    // enable/disable
    settings.gyro.enabled = 1;
    settings.gyro.enableX = 1;
    settings.gyro.enableY = 1;
    settings.gyro.enableZ = 1;

    // gyro scale can be 245, 500, or 2000 (degree/sec)
    settings.gyro.scale = 245;

    // gyro sample rate: value between 1-6
    // 1 = 14.9    4 = 238
    // 2 = 59.5    5 = 476
    // 3 = 119     6 = 952
    settings.gyro.sampleRate = 6;

    // gyro cutoff frequency: value between 0-3
    // Actual value of cutoff frequency depends
    // on sample rate.
    settings.gyro.bandwidth = 0;

    settings.gyro.lowPowerEnable = 0;

    settings.gyro.HPFEnable = 0;

    // Gyro HPF cutoff frequency: value between 0-9
    // Actual value depends on sample rate. Only applies
    // if gyroHPFEnable is true.
    settings.gyro.HPFCutoff = 0;
    settings.gyro.flipX = 0;
    settings.gyro.flipY = 0;
    settings.gyro.flipZ = 0;
    settings.gyro.orientation = 0;
    settings.gyro.latchInterrupt = 1;

    //-----------------------
    // 加速度センサ設定
    //-----------------------
    // enable/disable
    settings.accel.enabled = 0;
    settings.accel.enableX = 0;
    settings.accel.enableY = 0;
    settings.accel.enableZ = 0;

    // Accelerometer scale can be 2, 4, 8, or 16
    settings.accel.scale = aScl; //TODO: aScaleリプレース用

    // Accelerometer sample rate can be 1-6
    // 1 = 10 Hz    4 = 238 Hz
    // 2 = 50 Hz    5 = 476 Hz
    // 3 = 119 Hz   6 = 952 Hz
    settings.accel.sampleRate = 6;

    // Accelerometer cutoff frequency can be any value between -1 - 3.
    // -1 = bandwidth determined by sample rate
    // 0 = 408 Hz   2 = 105 Hz
    // 1 = 211 Hz   3 = 50 Hz
    settings.accel.bandwidth = -1;

    settings.accel.highResEnable = 0;

    // highResBandwidth can be any value between 0-3
    // LP cutoff is set to a factor of sample rate
    // 0 = ODR/50    2 = ODR/9
    // 1 = ODR/100   3 = ODR/400
    settings.accel.highResBandwidth = 0;

    //-----------------------
    // 磁気センサ設定
    //-----------------------
    // enable/disable
    settings.mag.enabled = 1;

    // mag scale can be 4, 8, 12, or 16
    settings.mag.scale = 4;

    // mag data rate can be 0-7
    // 0 = 0.625 Hz  4 = 10 Hz
    // 1 = 1.25 Hz   5 = 20 Hz
    // 2 = 2.5 Hz    6 = 40 Hz
    // 3 = 5 Hz      7 = 80 Hz
    settings.mag.sampleRate = 7;
    settings.mag.tempCompensationEnable = 0;
    
    // magPerformance can be any value between 0-3
    // 0 = Low power mode      2 = high performance
    // 1 = medium performance  3 = ultra-high performance
    settings.mag.XYPerformance = 3;
    settings.mag.ZPerformance = 3;
    settings.mag.lowPowerEnable = 0;
    
    // magOperatingMode can be 0-2
    // 0 = continuous conversion
    // 1 = single-conversion
    // 2 = power down
    settings.mag.operatingMode = 0;

    // バイアス（ゆらぎ値）格納用変数の初期化
    settings.temp.enabled = 1;
    for (int i=0; i<3; i++)
    {
        gBias[i] = 0;
        aBias[i] = 0;
        mBias[i] = 0;
        gBiasRaw[i] = 0;
        aBiasRaw[i] = 0;
        mBiasRaw[i] = 0;
    }
    
    // バイアス補正の使用するか(0:使用しない, 1:使用する)
    _autoCalc = 1;

    // スケールを変数に保存
    // Store the given scales in class variables. These scale variables
    // are used throughout to calculate the actual g's, DPS,and Gs's.
    aScale = aScl; //MEMO: initAccel()内部で初期スケールとして使用される
    gScale = gScl; //MEMO: initGyro() 内部で初期スケールとして使用される
    mScale = mScl; //MEMO: initMag()  内部で初期スケールとして使用される

    // Once we have the scale values, we can calculate the resolution
    // of each sensor. That's what these functions are for. One for each sensor
    calcaRes(); // Calculate g / ADC tick, stored in aRes variable
    calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
    calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable

    // Accelerometer initialization stuff:
    initAccel(); // "Turn on" all axes of the accelerometer. Set up interrupts, etc.
    setAccelODR(aODR); // Set the accelerometer data rate. (initAccel() の初期値以外に変更する場合)

    // Gyro initialization stuff:
    initGyro(); // This will "turn on" the gyro. Setting up interrupts, etc.
    setGyroODR(gODR); // Set the gyro output data rate and bandwidth.

    // Magnetometer initialization stuff:
    initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
    setMagODR(mODR); // Set the magnetometer output data rate.
}

// BEGIN ----------------------------------------------------------------------

// データの取得開始。事前に init() がコールされている想定
uint16_t GsLSM9DS1::begin()
{    
    // 変な値が入っていた時に修正してくれるヘルパー（不要）
    //constrainScales();

    // Once we have the scale values, we can calculate the resolution
    // of each sensor. That's what these functions are for. One for each sensor
    calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
    calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
    calcaRes(); // Calculate g / ADC tick, stored in aRes variable
    
    // I2C/SPI 初期化（不要）
    // Now, initialize our hardware interface.
//    if (settings.device.commInterface == IMU_MODE_I2C)  // If we're using I2C
//        initI2C();  // Initialize I2C
//    else if (settings.device.commInterface == IMU_MODE_SPI)     // else, if we're using SPI
//        initSPI();  // Initialize SPI
        
    // To verify communication, we can read from the WHO_AM_I register of
    // each device. Store those in a variable so we can return them.
    uint8_t mTest = mReadByte(WHO_AM_I_M);      // Read the gyro WHO_AM_I
    uint8_t agTest = agReadByte(WHO_AM_I_XG);   // Read the accel/mag WHO_AM_I
    uint16_t whoAmICombined = (agTest << 8) | mTest;
    
    if (whoAmICombined != ((AG_WHO_AM_I_VAL << 8) | M_WHO_AM_I_VAL))
        return 0; // error
    
    // Gyro initialization stuff:
    initGyro(); // This will "turn on" the gyro. Setting up interrupts, etc.
    
    // Accelerometer initialization stuff:
    initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
    
    // Magnetometer initialization stuff:
    initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

    // Once everything is initialized, return the WHO_AM_I registers we read:
    return whoAmICombined;
}

// Check -----------------------------------------------------------------------
/**
 * 加速度（ジャイロ）センサと磁気センサの WHO_AM_I 値を連結して返す
 * @return
 */
uint16_t GsLSM9DS1::checkWhoAmI()
{
    uint8_t agTest = i2cRead(agAddress, AG_WHO_AM_I);
    uint8_t mTest  = i2cRead(mAddress,  M_WHO_AM_I);
        
    return (( agTest << 8 ) | mTest ); // returns 0x683D (26685) if success
}

uint8_t GsLSM9DS1::accelAvailable()
{
    uint8_t status = agReadByte(STATUS_REG_1);
    
    return (uint8_t)(status & (1<<0));
}

uint8_t GsLSM9DS1::gyroAvailable()
{
    uint8_t status = agReadByte(STATUS_REG_1);
    
    return (uint8_t)((status & (1<<1)) >> 1);
}

uint8_t GsLSM9DS1::tempAvailable()
{
    uint8_t status = agReadByte(STATUS_REG_1);
    
    return (uint8_t)((status & (1<<2)) >> 2);
}

uint8_t GsLSM9DS1::magAvailable(lsm9ds1_axis axis)
{
    uint8_t status;
    status = mReadByte(STATUS_REG_M);
    
    return (uint8_t)((status & (1<<axis)) >> axis);
}

// Calibration -----------------------------------------------------------------

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void GsLSM9DS1::calibrate(bool autoCalc)
{
    int i;
    uint8_t samples = 0;
    int32_t aBiasRawTemp[3] = {0, 0, 0};
    int32_t gBiasRawTemp[3] = {0, 0, 0};

    _autoCalc = 0; // キャリブレーション中は自動補正を無効にする

    // Turn on FIFO and set threshold to 32 samples
    enableFIFO(true);
    setFIFO(FIFO_THS, 0x1F);
    
    while (samples < 0x1F)
    {
        samples = uint8_t(agReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
    }

    for(i = 0; i < samples ; i++)
    {   // Read the gyro data stored in the FIFO
        readGyro();
        gBiasRawTemp[X_AXIS] += gx_raw;
        gBiasRawTemp[Y_AXIS] += gy_raw;
        gBiasRawTemp[Z_AXIS] += gz_raw;
        readAccel();
        aBiasRawTemp[X_AXIS] += ax_raw;
        aBiasRawTemp[Y_AXIS] += ay_raw;
        //MEMO: 静止状態でも az は重力加速度(= az_raw * aRes = 1.0f)を受けているので、重力加速度成分を除去して格納する
        aBiasRawTemp[Z_AXIS] += az_raw - (int16_t)(1.0f / aRes); // Assumes sensor facing up!
    }

//    serial->printf("gyroBiasRawTemp %d/%d/%d\r\n", (int)gBiasRawTemp[X_AXIS], (int)gBiasRawTemp[Y_AXIS], (int)gBiasRawTemp[Z_AXIS]);
//    serial->printf("acclBiasRawTemp %d/%d/%d\r\n", (int)aBiasRawTemp[X_AXIS], (int)aBiasRawTemp[Y_AXIS], (int)aBiasRawTemp[Z_AXIS]);

    // ジャイロと加速度のゆらぎを求め、オフセットとして格納する
    for (i = 0; i < 3; i++) // x, y, z
    {
        // gyro
        gBiasRaw[i] = (int16_t) gBiasRawTemp[i] / samples;
        gBias[i] = calcGyro(gBiasRaw[i]);

        // accel
        aBiasRaw[i] = (int16_t) aBiasRawTemp[i] / samples;
        aBias[i] = calcAccel(aBiasRaw[i]);
    }

//    serial->printf("gyroBias(Raw) %d(%f)/%d(%f),/%d(%f)\r\n"
//            , gBiasRaw[X_AXIS], gBias[X_AXIS], gBiasRaw[Y_AXIS], gBias[Y_AXIS], gBiasRaw[Z_AXIS], gBias[Z_AXIS]);
//    serial->printf("acclBias(Raw) %d(%f)/%d(%f),/%d(%f)\r\n"
//                   , aBiasRaw[X_AXIS], aBias[X_AXIS], aBiasRaw[Y_AXIS], aBias[Y_AXIS], aBiasRaw[Z_AXIS], aBias[Z_AXIS]);

    // disable FIFO
    enableFIFO(false);
    setFIFO(FIFO_OFF, 0x00);
    
    if (autoCalc) _autoCalc = 1;
}

// 静止状態で磁気センサのゆらぎを求め、オフセットとして格納する
void GsLSM9DS1::calibrateMag(bool loadIn)
{
    uint8_t i, j;
    int32_t magBiasRawSum[3] = {0, 0, 0};

    const uint8_t samples = 32;

    // 128サンプル取得する
    for (i=0; i<samples; i++) //MEMO: up to 255 samples
    {
        // 磁気センサに次のデータが現れるまで待つ
        while (!magAvailable())
            ;

        readMag();
        magBiasRawSum[X_AXIS] += mx_raw;
        magBiasRawSum[Y_AXIS] += my_raw;
        magBiasRawSum[Z_AXIS] += mz_raw;
    }

//    serial->printf("magBiasRawSum: %d/%d/%d\r\n", (int)magBiasRawSum[X_AXIS], (int)magBiasRawSum[Y_AXIS], (int)magBiasRawSum[Z_AXIS]);

    // オフセットを計算する
    for (j = 0; j < 3; j++)
    {
        mBiasRaw[j] = (int16_t)magBiasRawSum[j] / samples;
        mBias[j] = calcMag(mBiasRaw[j]);

        // オフセットをレジスタに格納する
        if (loadIn) {
            magOffset(j, mBiasRaw[j]);
        }
    }

//    serial->printf("magBias(Raw) %f(%d)/%f(%d)/%f(%d)\r\n"
//                   , mBias[X_AXIS], mBiasRaw[X_AXIS] , mBias[Y_AXIS], mBiasRaw[Y_AXIS] , mBias[Z_AXIS], mBiasRaw[Z_AXIS]);
}

// 磁気センサのオフセット値をレジスタに格納する
void GsLSM9DS1::magOffset(uint8_t axis, int16_t offset)
{
    if (axis > 2)
        return;
    uint8_t msb, lsb;
    msb = (uint8_t)(offset & 0xFF00) >> 8;
    lsb = (uint8_t)(offset & 0x00FF);
    mWriteByte((uint8_t)(OFFSET_X_REG_L_M + (2 * axis)), lsb);
    mWriteByte((uint8_t)(OFFSET_X_REG_H_M + (2 * axis)), msb);
}

// FIFO  ----------------------------------------------------------------------

void GsLSM9DS1::enableFIFO(bool enable)
{
    uint8_t temp = agReadByte(CTRL_REG9);
    if (enable) temp |= (1<<1);
    else temp &= ~(1<<1);
    agWriteByte(CTRL_REG9, temp);
}

void GsLSM9DS1::setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
    // Limit threshold - 0x1F (31) is the maximum. If more than that was asked
    // limit it to the maximum.
    uint8_t threshold = (uint8_t)(fifoThs <= 0x1F ? fifoThs : 0x1F);
    agWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t GsLSM9DS1::getFIFOSamples()
{
    return uint8_t (agReadByte(FIFO_SRC) & 0x3F);
}


// Read  ----------------------------------------------------------------------

uint8_t GsLSM9DS1::agReadByte(uint8_t subAddress)
{        
    return I2CreadByte(agAddress, subAddress);
}

void GsLSM9DS1::agReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    I2CreadBytes(agAddress, subAddress, dest, count);
}

uint8_t GsLSM9DS1::mReadByte(uint8_t subAddress)
{
    return I2CreadByte(mAddress, subAddress);
}

/*
void GsLSM9DS1::readAccel()
{
    // The data we are going to read from the accel
    char data[6];
 
    // The start of the addresses we want to read from
    char subAddress = A_OUT_X_L;
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(agAddress, &subAddress, 1, true);
    // Read in all 8 bit registers containing the axes data
    i2c.read(agAddress, data, 6);
 
    // Reassemble the data and convert to g
    ax_raw = data[0] | (data[1] << 8);
    ay_raw = data[2] | (data[3] << 8);
    az_raw = data[4] | (data[5] << 8);
    ax = ax_raw * aRes;
    ay = ay_raw * aRes;
    az = az_raw * aRes;
}
*/

/**
 * 加速度センサの値を読み込む。
 * _autoCals が有効な場合、ゆらぎ補正も行う
 */
void GsLSM9DS1::readAccel()
{
    uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
    agReadBytes(OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL

    // store raw values
    ax_raw = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
    ay_raw = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
    az_raw = (temp[5] << 8) | temp[4]; // Store z-axis values into az

    // _autoCalc が有効な場合、ゆらぎ補正を行う
    if (_autoCalc)
    {
        ax_raw -= aBiasRaw[X_AXIS];
        ay_raw -= aBiasRaw[Y_AXIS];
        az_raw -= aBiasRaw[Z_AXIS];
    }

    // calc real-world values
    ax = ax_raw * aRes;
    ay = ay_raw * aRes;
    az = az_raw * aRes;

//    serial->printf("Acc: %6.3f/%6.3f/%6.3f\r\n", ax, ay, az);
}

void GsLSM9DS1::readGyro()
{
    uint8_t temp[6]; // We'll read six bytes from the gyro into temp
    
    agReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G

    // store raw values
    gx_raw = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
    gy_raw = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
    gz_raw = (temp[5] << 8) | temp[4]; // Store z-axis values into gz

    // _autoCalc が有効な場合、ゆらぎ補正を行う
    if (_autoCalc)
    {
        gx_raw -= gBiasRaw[X_AXIS];
        gy_raw -= gBiasRaw[Y_AXIS];
        gz_raw -= gBiasRaw[Z_AXIS];
    }

    //MEMO: 取得値を dps (degree per sec: 度/秒)に変換する
    // calc real-world values
    gx = gx_raw * gRes;
    gy = gy_raw * gRes;
    gz = gz_raw * gRes;

//    serial->printf("Gyro: %6.3f/%6.3f/%6.3f\r\n", gx, gy, gz);
}

void GsLSM9DS1::readMag()
{
    // The data we are going to read from the mag
    char data[6];
 
    // The start of the addresses we want to read from
    char subAddress = M_OUT_X_L;
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(mAddress, &subAddress, 1, true);

    // Read in all 8 bit registers containing the axes data
    i2c.read(mAddress, data, 6);
 
    // Reassemble the data and convert to degrees
    mx_raw = data[0] | (data[1] << 8);
    my_raw = data[2] | (data[3] << 8);
    mz_raw = data[4] | (data[5] << 8);

    // _autoCalc が有効な場合、ゆらぎ補正を行う
    if (_autoCalc)
    {
        mx_raw -= mBiasRaw[X_AXIS];
        my_raw -= mBiasRaw[Y_AXIS];
        mz_raw -= mBiasRaw[Z_AXIS];
    }

    mx = mx_raw * mRes;
    my = my_raw * mRes;
    mz = mz_raw * mRes;

//    serial->printf("Mag(Raw): %f(%d)/%f(%d)/%f(%d)\r\n", mx, mx_raw, my, my_raw, mz, mz_raw);
}

/**
 * 温度センサの値を取得して変数に格納する
 * 温度センサは int16_t の範囲で、25℃ のとき 0 を返す
 * Vdd=2.2v, T=25℃ のとき、16/℃ で変化する
 */
void GsLSM9DS1::readTemp()
{
    // The data we are going to read from the temp
    char data[2];
 
    // The start of the addresses we want to read from
    char subAddress = OUT_TEMP_L;
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(agAddress, &subAddress, 1, true);

    // Read in all 8 bit registers containing the axes data
    i2c.read(agAddress, data, 2);
 
    // Temperature is stored in int16_t but actually a 12-bit signed integer
    temperature_raw = data[0] | (data[1] << 8);
 
    temperature_c = (float)temperature_raw / 16.0f + 25.0f; // temperature_raw=0 @25C

//    serial->printf("Temp(Raw): %f(%d)\r\n", temperature_c, temperature_raw);
}

// Write ----------------------------------------------------------------------

void GsLSM9DS1::agWriteByte(uint8_t subAddress, uint8_t data)
{
    // Whether we're using I2C or SPI, write a byte using the
    // gyro-specific I2C address or SPI CS pin.
//    if (settings.device.commInterface == IMU_MODE_I2C) {
//        printf("yo");
//        I2CwriteByte(_xgAddress, subAddress, data);
//    } else if (settings.device.commInterface == IMU_MODE_SPI) {
//        SPIwriteByte(_xgAddress, subAddress, data);
//    }

    I2CwriteByte(agAddress, subAddress, data);
}

void GsLSM9DS1::mWriteByte(uint8_t subAddress, uint8_t data)
{
    // Whether we're using I2C or SPI, write a byte using the
    // accelerometer-specific I2C address or SPI CS pin.
//    if (settings.device.commInterface == IMU_MODE_I2C)
//        return I2CwriteByte(_mAddress, subAddress, data);
//    else if (settings.device.commInterface == IMU_MODE_SPI)
//        return SPIwriteByte(_mAddress, subAddress, data);

    I2CwriteByte(agAddress, subAddress, data);
}


// I2C ------------------------------------------------------------------------

uint8_t GsLSM9DS1::i2cRead(int address, char subAddress)
{
    cmd[0] = subAddress;
    i2c.write(address, cmd, 1, true);
    i2c.read(address, cmd+1, 1);
        
    return (uint8_t) cmd[1];
}

uint8_t GsLSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
    char data;
    char temp[1] = {subAddress};
    
    i2c.write(address, temp, 1); // address, data, data-length
    temp[1] = 0x00;
    i2c.write(address, temp, 1);
    i2c.read(address, &data, 1);
    return (uint8_t) data;
}

uint8_t GsLSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{  
    int i;
    char temp_dest[count];
    char temp[1] = {subAddress};
    i2c.write(address, temp, 1);
    i2c.read(address, temp_dest, count);
    
    //i2c doesn't take uint8_ts, but rather chars so do this nasty af conversion
    for (i=0; i < count; i++) {
        dest[i] = temp_dest[i];    
    }
    return count;
}

// Wire.h read and write protocols
void GsLSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    /* 
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
    */
    char temp_data[2] = {subAddress, data};
    i2c.write(address, temp_data, 2);
}

// Calculation ----------------------------------------------------------------
/**
 * 加速度データの最小単位をセットする。最小単位は設定したSCALEによって異なる
 */
void GsLSM9DS1::calcaRes()
{
    // Possible accelerometer scales (and their register bit settings) are:
    // 2g (000), 4g (001), 6g (010) 8g (011), 16g (100).
    // Unit: mg/LSB
    // [-180:180](-pi:pi) = [-32768:32768]
    aRes = 0.0f;
    switch (aScale)
    {
        case A_SCALE_2G:
            aRes = 2.0f / 32768.0f;  // 0.000061
            break;
        case A_SCALE_4G:
            aRes = 4.0f / 32768.0f;  // 0.000122
            break;
        case A_SCALE_8G:
            aRes = 8.0f / 32768.0f;  // 0.000244
            break;
        case A_SCALE_16G:
            aRes = 16.0f / 32768.0f; // 0.000732
            break;
    }
//    serial->printf("aRes:%7.6f\r\n", aRes);
}

/**
 * ジャイロデータの最小単位をセットする。最小単位は gScale によって異なる
 */
void GsLSM9DS1::calcgRes()
{
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), 2000 DPS (10).
    // Unit: mdps/LSB
    // [-180:180](-pi:pi) = [-32768:32768]
    switch (gScale)
    {
        case G_SCALE_245DPS:
            gRes = 245.0f / 32768.0f;  // 0.007477
            break;
        case G_SCALE_500DPS:
            gRes = 500.0f / 32768.0f;  // 0.015259
            break;
        case G_SCALE_2000DPS:
            gRes = 2000.0f / 32768.0f; // 0.061035
            break;
    }
//    serial->printf("gRes:%7.6f\r\n", gRes); //MEMO: DEBUG ONLY
}

/**
 * 磁気データの最小単位をセットする。最小単位は設定したSCALEによって異なる
 */
void GsLSM9DS1::calcmRes()
{
    // Possible magnetometer scales (and their register bit settings) are:
    // +/-4Gs (00), +/-8Gs (01), +/-12Gs (10) +/-16Gs (11). 
    // [-max:+max] = [-32768:32768]
    switch (mScale)
    {
        case M_SCALE_4GS:
            mRes = 4.0f / 32768.0f;  // 0.000122
            break;
        case M_SCALE_8GS:
            mRes = 8.0f / 32768.0f;  // 0.000244
            break;
        case M_SCALE_12GS:
            mRes = 12.0f / 32768.0f; // 0.000366
            break;
        case M_SCALE_16GS:
            mRes = 16.0f / 32768.0f; // 0.000732
            break;
    }
//    serial->printf("mRes:%7.6f\r\n", mRes); //MEMO: DEBUG ONLY
}

float GsLSM9DS1::calcGyro(int16_t gyro)
{
    // Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
    return gRes * gyro; 
}

float GsLSM9DS1::calcAccel(int16_t accel)
{
    // Return the accel raw reading times our pre-calculated g's / (ADC tick):
    return aRes * accel;
}

float GsLSM9DS1::calcMag(int16_t mag)
{
    // Return the mag raw reading times our pre-calculated Gs / (ADC tick):
    return mRes * mag;
}

/**
 * 加速度センサ初期化:必要な設定を一括設定
 * 対象レジスタ: A_CTRL_REG5 (0x1F) & A_CTRL_REG6 (0x20) & A_CTRL_REG7 (0x21)
 * A_CTRL_REG5: [DEC_1][DEC_0][Zen_XL][Yen_XL] [Xen_XL][0][0][0]
 *       初期値: 0x38 (0011 1000)
 * A_CTRL_REG6: [ODR_XL2][ODR_XL1][ODR_XL0][FS_XL1] [FS_XL0][BW_SCAL_ODR][BW_XL1][BW_XL0]
 *       初期値: 0x60 (0110 0000)
 * A_CTRL_REG7: [HR][DCF1][DCF0][0] [0][FDS][0][HPIS1]
 *       初期値: 0x00 (0000 0000)
 */
void GsLSM9DS1::initAccel()
{
    // DEC_[0:1] Decimation(間引き) of acceleration data on OUT REG and FIFO.
    //    00: no decimation (default)
    //    01: update every 2 samples
    //    10: update every 4 samples
    //    11: update every 8 samples
    // Zen_XL Accelerometer's Z-axis output enable
    //    0: Z-axis output disabled
    //    1: Z-axis output enabled (default)
    // Yen_XL Accelerometer's Y-axis output enable
    //    0: Z-axis output disabled
    //    1: Z-axis output enabled (default)
    // Xen_XL Accelerometer's X-axis output enable
    //    0: Z-axis output disabled
    //    1: Z-axis output enabled (default)

    // ODR_XL[2:0] Output Data Rate (ODR) & Power Mode Selection
    //    000: Power Down (default)
    //    001: 10Hz
    //    010: 50Hz
    //    011: 119Hz
    //    100: 238Hz
    //    101: 476Hz
    //    110: 952Hz
    //    111: N/A
    // FS_XL[1:0] Accelerometer full-scale selection
    //    00: +/- 2g (default)
    //    01: +/-16g
    //    10: +/- 4g
    //    11: +/- 8g
    // BW_SCAL_ODR Bandwidth (BW) selection
    //    0: BW determined by ODR selection
    //        BW = 408Hz when ODR = 952Hz, 50Hz, 10Hz
    //        BW = 211Hz when ODR = 476Hz
    //        BW = 105Hz when ODR = 238Hz
    //        Bw =  50Hz when ODR = 119Hz
    //    1: BW selected according to BW_XL[1:0] selection
    // BW_XL[1:0] Anti-aliasing filter bandwidth selection.
    //    00: 408Hz
    //    01: 211Hz
    //    10: 105Hz
    //    11:  50Hz

    // HR: High Resolution mode for accelerometer enable.
    //    0: disabled (default)
    //    1: enabled
    // DCF[1:0] Accelerometer digital filter (high-pass & low-pass) cutoff frequency selection.
    //          The bandwidth of the low-pass (LP) filter depends on the selected ODR.
    //    HR=1 & DCF[1:0]=00 -> LP cutoff freq =  50Hz
    //    HR=1 & DCF[1:0]=01 -> LP cutoff freq = 100Hz
    //    HR=1 & DCF[1:0]=10 -> LP cutoff freq =   9Hz
    //    HR=1 & DCF[1:0]=11 -> LP cutoff freq = 400Hz
    // FDS: Filtered data selection
    //    0: Internal filter bypassed
    //    1: Data from internal filter sent to output register and FIFO
    // HPIS1 High-pass filter enabled for acceleration sensor interrupt function on Interrupt
    //    0: filter bypassed (default)
    //    1: filter enabled

    char cmd[4] = {
        A_CTRL_REG5,    // start address (0x1F)
        0x38,           // 0x38 @ 0x1F: Enable all axis and don't decimate data
        (A_ODR_119 << 5) | (aScale << 3) | (A_BW_AUTO_SCALE << 2),   // 0x60 @ 0x20: 119Hz ODR, 50Hz Bandwidth
        0               // 0x00 @ 0x21: Disable High Resolution mode, filter bypassed
    };
 
    // Write the data to the accel control registers
    i2c.write(agAddress, cmd, 4);
}

/**
 * Update Output Data Rate (ODR) & Power Mode Selection
 * @param aODR
 */
void GsLSM9DS1::setAccelODR(accel_odr aODR)
{
    // The start of the addresses we want to read from
    // 現在の設定を読み出すためのデータセット作成
    char cmd[2] = {
        A_CTRL_REG6, // start address (0x20)
        0
    };
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(agAddress, cmd, 1, true);

    // Read in all the 8 bits of data from 0x20 into cmd[1]
    i2c.read(agAddress, cmd+1, 1);
 
    // Then mask out the accel odr bits:
    cmd[1] &= 0xFF^(0x7 << 5);
    // Then shift in our new odr bits:
    cmd[1] |= aODR << 5;
 
    // Write the ODR out to the address
    i2c.write(agAddress, cmd, 2);
}

void GsLSM9DS1::setAccelScale(accel_scale aScl)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        A_CTRL_REG6,
        0
    };
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(agAddress, cmd, 1, true);

    // Read in all the 8 bits of data
    i2c.read(agAddress, cmd+1, 1);
 
    // Then mask out the accel scale bits:
    cmd[1] &= 0xFF^(0x3 << 3);
    // Then shift in our new scale bits:
    cmd[1] |= aScl << 3;
 
    // Write the accelscale out to the accel
    i2c.write(agAddress, cmd, 2);
    
    // We've updated the sensor, but we also need to update our class variables
    // First update aScale:
    aScale = aScl;
    
    // Then calculate a new aRes, which relies on aScale being set correctly:
    calcaRes();
}

/**
 * ジャイロセンサ初期化:必要な設定を一括設定
 * 対象レジスタ: G_CTRL_REG1 (0x10) & G_CTRL_REG2 (0x11) & G_CTRL_REG3 (0x12)
 * G_CTRL_REG1: [ODR_G2][ODR_G1][ODR_G0][FS_G1] [FS_G0][0][BW_G1][GW_G0]
 *       初期値:
 * G_CTRL_REG2: [0][0][0][0] [INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
 * G_CTRL_REG3: [LP_mode][HP_EN][0][0] [HPCF_G3][HPCF_G2][HPCF_G1][HPCF_G0]
 */
void GsLSM9DS1::initGyro()
{
    // ODR_G[2:0] Gyroscope Output Data Rate (ODR) selection
    //    000: Power-down (default)
    //    001: ODR= 14.9Hz, Cutoff=  5Hz
    //    010: ODR= 59.5Hz, Cutoff= 19Hz
    //    011: ODR=119.0Hz, Cutoff= 38Hz
    //    100: ODR=238.0Hz, Cutoff= 76Hz
    //    101: ODR=476.0Hz, Cutoff=100Hz
    //    110: ODR=952.0Hz, Cutoff=100Hz
    //    111: N/A
    // FS_G[1:0] Gyroscope full-scale selection
    //    00:  245dps (default)
    //    01:  500dps
    //    10:  N/A
    //    11: 2000dps
    // BW_G[1:0] Gyroscope bandwidth (BW) selection
    //    ODR_G[2:0]=000 & BW_G[1:0]=00 ->

    char cmd[4] = {
        G_CTRL_REG1,
        gScale | G_ODR_119_BW_14,   // @0x10
        0,          // @0x11: Default data INT & OUT selection
        0           // @0x12: Default power mode and High-pass settings
    };
 
    // Write the data to the gyro control registers
    i2c.write(agAddress, cmd, 4);
}

void GsLSM9DS1::setGyroODR(gyro_odr gODR)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        G_CTRL_REG1, // 0x10
        0
    };
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(agAddress, cmd, 1, true);

    // Read in all the 8 bits of data
    i2c.read(agAddress, cmd+1, 1);
 
    // Then mask out the gyro odr bits:
    cmd[1] &= (0x3 << 3);

    // Then shift in our new odr bits:
    cmd[1] |= gODR;
 
    // Write the gyroodr out to the gyro
    i2c.write(agAddress, cmd, 2);
}

void GsLSM9DS1::setGyroScale(gyro_scale gScl)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        G_CTRL_REG1,
        0
    };
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(agAddress, cmd, 1, true);
    // Read in all the 8 bits of data
    i2c.read(agAddress, cmd+1, 1);
 
    // Then mask out the gyro scale bits:
    cmd[1] &= 0xFF^(0x3 << 3);
    // Then shift in our new scale bits:
    cmd[1] |= gScl << 3;
 
    // Write the gyroscale out to the gyro
    i2c.write(agAddress, cmd, 2);
    
    // We've updated the sensor, but we also need to update our class variables
    // First update gScale:
    gScale = gScl;
    
    // Then calculate a new gRes, which relies on gScale being set correctly:
    calcgRes();
}

void GsLSM9DS1::initMag()
{   
    char cmd[4] = {
        M_CTRL_REG1,
        0x10,       // Default data rate, xy axes mode, and temp comp
        mScale << 5,// Set mag scale
        0           // Enable I2C, write only SPI, not LP mode, Continuous conversion mode
    };
 
    // Write the data to the mag control registers
    i2c.write(mAddress, cmd, 4);
}

void GsLSM9DS1::setMagODR(mag_odr mODR)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        M_CTRL_REG1,
        0
    };
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(mAddress, cmd, 1, true);
    // Read in all the 8 bits of data
    i2c.read(mAddress, cmd+1, 1);
 
    // Then mask out the mag odr bits:
    cmd[1] &= 0xFF^(0x7 << 2);
    // Then shift in our new odr bits:
    cmd[1] |= mODR << 2;
 
    // Write the magodr out to the mag
    i2c.write(mAddress, cmd, 2);
}

void GsLSM9DS1::setMagScale(mag_scale mScl)
{
    // The start of the addresses we want to read from
    char cmd[2] = {
        M_CTRL_REG2,
        0
    };
 
    // Write the address we are going to read from and don't end the transaction
    i2c.write(mAddress, cmd, 1, true);
    // Read in all the 8 bits of data
    i2c.read(mAddress, cmd+1, 1);
 
    // Then mask out the mag scale bits:
    cmd[1] &= 0xFF^(0x3 << 5);
    // Then shift in our new scale bits:
    cmd[1] |= mScl << 5;
 
    // Write the magscale out to the mag
    i2c.write(mAddress, cmd, 2);
    
    // We've updated the sensor, but we also need to update our class variables
    // First update mScale:
    mScale = mScl;
    
    // Then calculate a new mRes, which relies on mScale being set correctly:
    calcmRes();
}
#endif