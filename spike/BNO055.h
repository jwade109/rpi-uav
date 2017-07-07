#ifndef BNO055_H
#define BNO055_H

#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <stdint.h>
#include <I2C.h>

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID        (0xA0)

#define NUM_BNO055_OFFSET_REGISTERS (22)

typedef struct
{
    uint16_t accel_offset_x;
    uint16_t accel_offset_y;
    uint16_t accel_offset_z;
    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;
    uint16_t mag_offset_x;
    uint16_t mag_offset_y;
    uint16_t mag_offset_z;

    uint16_t accel_radius;
    uint16_t mag_radius;
}
adafruit_bno055_offsets_t;

typedef enum
{
    /* Page id register definition */
    BNO055_PAGE_ID_ADDR                                     = 0x07,

    /* PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR                                     = 0x00,
    BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
    BNO055_MAG_REV_ID_ADDR                                  = 0x02,
    BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
    BNO055_BL_REV_ID_ADDR                                   = 0x06,

    /* Accel data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0x08,
    BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0x09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0x0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0x0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0x0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0x0D,

    /* Mag data register */
    BNO055_MAG_DATA_X_LSB_ADDR                              = 0x0E,
    BNO055_MAG_DATA_X_MSB_ADDR                              = 0x0F,
    BNO055_MAG_DATA_Y_LSB_ADDR                              = 0x10,
    BNO055_MAG_DATA_Y_MSB_ADDR                              = 0x11,
    BNO055_MAG_DATA_Z_LSB_ADDR                              = 0x12,
    BNO055_MAG_DATA_Z_MSB_ADDR                              = 0x13,

    /* Gyro data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR                             = 0x14,
    BNO055_GYRO_DATA_X_MSB_ADDR                             = 0x15,
    BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0x16,
    BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0x17,
    BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0x18,
    BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0x19,

    /* Euler data registers */
    BNO055_EULER_H_LSB_ADDR                                 = 0x1A,
    BNO055_EULER_H_MSB_ADDR                                 = 0x1B,
    BNO055_EULER_R_LSB_ADDR                                 = 0x1C,
    BNO055_EULER_R_MSB_ADDR                                 = 0x1D,
    BNO055_EULER_P_LSB_ADDR                                 = 0x1E,
    BNO055_EULER_P_MSB_ADDR                                 = 0x1F,

    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0x20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0x21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0x22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0x23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0x24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0x25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0x26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0x27,

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0x28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0x29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0x2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0x2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0x2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0x2D,

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0x2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0x2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0x30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0x31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0x32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0x33,

    /* Temperature data register */
    BNO055_TEMP_ADDR                                        = 0x34,

    /* Status registers */
    BNO055_CALIB_STAT_ADDR                                  = 0x35,
    BNO055_SELFTEST_RESULT_ADDR                             = 0x36,
    BNO055_INTR_STAT_ADDR                                   = 0x37,

    BNO055_SYS_CLK_STAT_ADDR                                = 0x38,
    BNO055_SYS_STAT_ADDR                                    = 0x39,
    BNO055_SYS_ERR_ADDR                                     = 0x3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR                                    = 0x3B,
    BNO055_DATA_SELECT_ADDR                                 = 0x3C,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR                                    = 0x3D,
    BNO055_PWR_MODE_ADDR                                    = 0x3E,

    BNO055_SYS_TRIGGER_ADDR                                 = 0x3F,
    BNO055_TEMP_SOURCE_ADDR                                 = 0x40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR                             = 0x41,
    BNO055_AXIS_MAP_SIGN_ADDR                               = 0x42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0x43,
    BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0x44,
    BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0x45,
    BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0x46,
    BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0x47,
    BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0x48,
    BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0x49,
    BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0x4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0x4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0x4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0x4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0x4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0x4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0x50,
    BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0x51,
    BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0x52,
    BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0x53,
    BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0x54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR                                 = 0x55,
    ACCEL_OFFSET_X_MSB_ADDR                                 = 0x56,
    ACCEL_OFFSET_Y_LSB_ADDR                                 = 0x57,
    ACCEL_OFFSET_Y_MSB_ADDR                                 = 0x58,
    ACCEL_OFFSET_Z_LSB_ADDR                                 = 0x59,
    ACCEL_OFFSET_Z_MSB_ADDR                                 = 0x5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR                                   = 0x5B,
    MAG_OFFSET_X_MSB_ADDR                                   = 0x5C,
    MAG_OFFSET_Y_LSB_ADDR                                   = 0x5D,
    MAG_OFFSET_Y_MSB_ADDR                                   = 0x5E,
    MAG_OFFSET_Z_LSB_ADDR                                   = 0x5F,
    MAG_OFFSET_Z_MSB_ADDR                                   = 0x60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR                                  = 0x61,
    GYRO_OFFSET_X_MSB_ADDR                                  = 0x62,
    GYRO_OFFSET_Y_LSB_ADDR                                  = 0x63,
    GYRO_OFFSET_Y_MSB_ADDR                                  = 0x64,
    GYRO_OFFSET_Z_LSB_ADDR                                  = 0x65,
    GYRO_OFFSET_Z_MSB_ADDR                                  = 0x66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR                                   = 0x67,
    ACCEL_RADIUS_MSB_ADDR                                   = 0x68,
    MAG_RADIUS_LSB_ADDR                                     = 0x69,
    MAG_RADIUS_MSB_ADDR                                     = 0x6A
}
adafruit_bno055_reg_t;

typedef enum
{
    POWER_MODE_NORMAL                                       = 0x00,
    POWER_MODE_LOWPOWER                                     = 0x01,
    POWER_MODE_SUSPEND                                      = 0x02
}
adafruit_bno055_powermode_t;

typedef enum
{
    /* Operation mode settings*/
    OPERATION_MODE_CONFIG                                   = 0x00,
    OPERATION_MODE_ACCONLY                                  = 0x01,
    OPERATION_MODE_MAGONLY                                  = 0x02,
    OPERATION_MODE_GYRONLY                                  = 0x03,
    OPERATION_MODE_ACCMAG                                   = 0x04,
    OPERATION_MODE_ACCGYRO                                  = 0x05,
    OPERATION_MODE_MAGGYRO                                  = 0x06,
    OPERATION_MODE_AMG                                      = 0x07,
    OPERATION_MODE_IMUPLUS                                  = 0x08,
    OPERATION_MODE_COMPASS                                  = 0x09,
    OPERATION_MODE_M4G                                      = 0x0A,
    OPERATION_MODE_NDOF_FMC_OFF                             = 0x0B,
    OPERATION_MODE_NDOF                                     = 0x0C
}
adafruit_bno055_opmode_t;

typedef enum
{
    REMAP_CONFIG_P0                                         = 0x21,
    REMAP_CONFIG_P1                                         = 0x24, // default
    REMAP_CONFIG_P2                                         = 0x24,
    REMAP_CONFIG_P3                                         = 0x21,
    REMAP_CONFIG_P4                                         = 0x24,
    REMAP_CONFIG_P5                                         = 0x21,
    REMAP_CONFIG_P6                                         = 0x21,
    REMAP_CONFIG_P7                                         = 0x24
}
adafruit_bno055_axis_remap_config_t;

typedef enum
{
    REMAP_SIGN_P0                                           = 0x04,
    REMAP_SIGN_P1                                           = 0x00, // default
    REMAP_SIGN_P2                                           = 0x06,
    REMAP_SIGN_P3                                           = 0x02,
    REMAP_SIGN_P4                                           = 0x03,
    REMAP_SIGN_P5                                           = 0x01,
    REMAP_SIGN_P6                                           = 0x07,
    REMAP_SIGN_P7                                           = 0x05
}
adafruit_bno055_axis_remap_sign_t;

typedef struct
{
    uint8_t  accel_rev;
    uint8_t  mag_rev;
    uint8_t  gyro_rev;
    uint16_t sw_rev;
    uint8_t  bl_rev;
}
adafruit_bno055_rev_info_t;

typedef enum
{
    VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
    VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
    VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
    VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
}
adafruit_vector_type_t;

class BNO055
{
    public:

        BNO055();
        bool begin(uint8_t addr = BNO055_ADDRESS_A,
            adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
        void setMode(adafruit_bno055_opmode_t mode);
        void getRevInfo(adafruit_bno055_rev_info_t*);
        void displayRevInfo(void);
        void setExtCrystalUse(boolean usextal);
        void getSystemStatus(uint8_t *system_status,
            uint8_t *self_test_result, uint8_t *system_error);
        void displaySystemStatus(void);
        uint8_t getCalibration(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

        imu::Vector<3> getVector(adafruit_vector_type_t vector_type );
        imu::Quaternion getQuat(void);
        int8_t getTemp(void);

        /* Functions to deal with raw calibration data */
        bool getSensorOffsets(uint8_t* calibData);
        bool getSensorOffsets(adafruit_bno055_offsets_t &offsets_type);
        void setSensorOffsets(const uint8_t* calibData);
        void setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type);
        bool isFullyCalibrated(void);

    private:

        bool readLen(adafruit_bno055_reg_t, byte* buffer, uint8_t len);

        I2C i2c;
        adafruit_bno055_opmode_t opmode;
};

#endif
