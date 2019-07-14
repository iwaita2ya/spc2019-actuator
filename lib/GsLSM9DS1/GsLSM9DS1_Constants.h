#ifndef __GsLSM9DS1_Constants_H__
#define __GsLSM9DS1_Constants_H__

// Temperature ----------------------------------------------------------------
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16

// Gyroscope ------------------------------------------------------------------
#define AG_WHO_AM_I         0x0F
#define AG_WHO_AM_I_VAL     0x68

#define A_CTRL_REG5         0x1F
#define A_CTRL_REG6         0x20
#define A_CTRL_REG7         0x21

#define A_OUT_X_L           0x28
#define A_OUT_X_H           0x29
#define A_OUT_Y_L           0x2A
#define A_OUT_Y_H           0x2B
#define A_OUT_Z_L           0x2C
#define A_OUT_Z_H           0x2D

// Gyroscope ------------------------------------------------------------------
#define G_CTRL_REG1         0x10
#define G_CTRL_REG2         0x11
#define G_CTRL_REG3         0x12
#define G_ORIENT_CFG        0x13

#define G_OUT_X_L           0x18
#define G_OUT_X_H           0x19
#define G_OUT_Y_L           0x1A
#define G_OUT_Y_H           0x1B
#define G_OUT_Z_L           0x1C
#define G_OUT_Z_H           0x1D

// Magnetometer ---------------------------------------------------------------
#define M_CTRL_REG1         0x20
#define M_CTRL_REG2         0x21
#define M_CTRL_REG3         0x22
#define M_CTRL_REG4         0x23
#define M_CTRL_REG5         0x24

#define M_OUT_X_L           0x28
#define M_OUT_X_H           0x29
#define M_OUT_Y_L           0x2A
#define M_OUT_Y_H           0x2B
#define M_OUT_Z_L           0x2C
#define M_OUT_Z_H           0x2D

#define M_WHO_AM_I          0x0F
#define M_WHO_AM_I_VAL      0x3D

#define M_V_ANGLE_SAPPORO   9.2f
#define M_V_ANGLE_TOKYO     7.0f
#define M_V_ANGLE_NAGOYA    7.2f
#define M_OFFSET_X          0x00
#define M_OFFSET_Y          0x00
#define M_OFFSET_Z          0x00

enum lsm9ds1_axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
};

// Sensor Sensitivity (Scale) -------------------------------------------------
// Lineear acceleration sensitivity
// write at A_CTRL_REG6 (20h)
enum accel_scale
{
    A_SCALE_2G,     // 00: +/- 2g
    A_SCALE_16G,    // 01: +/- 16g
    A_SCALE_4G,     // 10: +/- 4g
    A_SCALE_8G      // 11: +/- 8g
};

// Angular rate sensitivity FS_G[1:0]
// write on CTRL_REG1_G (10h)
enum gyro_scale
{
    G_SCALE_245DPS  = 0x0 << 3, // 00 << 3: +/-  245 degrees per second
    G_SCALE_500DPS  = 0x1 << 3, // 01 << 3: +/-  500 dps
    G_SCALE_2000DPS = 0x3 << 3  // 11 << 3: +/- 2000 dps
};


// Magnetic sensitivity
enum mag_scale
{
    M_SCALE_4GS,    // 00: +/- 4Gs
    M_SCALE_8GS,    // 01: +/- 8Gs
    M_SCALE_12GS,   // 10: +/- 12Gs
    M_SCALE_16GS,   // 11: +/- 16Gs
};

// Bandwidh -------------------------------------------------------------------

// accel_bw defines all possible bandwiths for low-pass filter of the accelerometer
// Used at A_CTRL_REG6 (0x20) Register
enum accel_bw
{
    A_BW_AUTO_SCALE = 0x0,  // Automatic BW scaling (0x0)
    A_BW_408 = 0x4,         // 408 Hz (0x4)
    A_BW_211 = 0x5,         // 211 Hz (0x5)
    A_BW_105 = 0x6,         // 105 Hz (0x6)
    A_BW_50  = 0x7          //  50 Hz (0x7)
};

// Output Data Rate -----------------------------------------------------------

// accel_oder defines all possible output data rates of the accelerometer:
// write on A_CTRL_REG6 (20h)
enum accel_odr
{
    A_POWER_DOWN,   // Power-down mode (0x0)
    A_ODR_10,       //  10 Hz (0x1)
    A_ODR_50,       //  50 Hz (0x2)
    A_ODR_119,      // 119 Hz (0x3)
    A_ODR_238,      // 238 Hz (0x4)
    A_ODR_476,      // 476 Hz (0x5)
    A_ODR_952       // 952 Hz (0x6)
};

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
// write the value on G_CTRL_REG1 (10h) register ODR_G[2:0]
enum gyro_odr
{                               // ODR(Hz) --- Cutoff
    G_POWER_DOWN     = 0x00,    // 0           0
    G_ODR_15_BW_0    = 0x20,    // 14.9        0
    G_ODR_60_BW_16   = 0x40,    // 59.5        16
    G_ODR_119_BW_14  = 0x60,    // 119         14
    G_ODR_119_BW_31  = 0x61,    // 119         31
    G_ODR_238_BW_14  = 0x80,    // 238         14
    G_ODR_238_BW_29  = 0x81,    // 238         29
    G_ODR_238_BW_63  = 0x82,    // 238         63
    G_ODR_238_BW_78  = 0x83,    // 238         78
    G_ODR_476_BW_21  = 0xA0,    // 476         21
    G_ODR_476_BW_28  = 0xA1,    // 476         28
    G_ODR_476_BW_57  = 0xA2,    // 476         57
    G_ODR_476_BW_100 = 0xA3,    // 476         100
    G_ODR_952_BW_33  = 0xC0,    // 952         33
    G_ODR_952_BW_40  = 0xC1,    // 952         40
    G_ODR_952_BW_58  = 0xC2,    // 952         58
    G_ODR_952_BW_100 = 0xC3     // 952         100
};

/// mag_odr defines all possible output data rates of the magnetometer:
enum mag_odr
{
    M_ODR_0625, // 0.625 Hz (0x00)
    M_ODR_125,  // 1.25 Hz  (0x01)
    M_ODR_25,   // 2.5 Hz   (0x02)
    M_ODR_5,    // 5 Hz     (0x03)
    M_ODR_10,   // 10       (0x04)
    M_ODR_20,   // 20 Hz    (0x05)
    M_ODR_40,   // 40 Hz    (0x06)
    M_ODR_80    // 80 Hz    (0x07)
};

enum fifoMode_type
{
    FIFO_OFF = 0,
    FIFO_THS = 1,
    FIFO_CONT_TRIGGER = 3,
    FIFO_OFF_TRIGGER = 4,
    FIFO_CONT = 5
};

    
#endif