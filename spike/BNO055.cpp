#include <math.h>
#include <BNO055.h>
#include <TimeUtil.h>

BNO055::BNO055()
{
    i2c = I2C(0);
}

bool BNO055::begin(uint8_t addr, adafruit_bno055_opmode_t mode)
{
    i2c = I2C(addr);
    if (!i2c.ready()) return false;
    /* Make sure we have the right device */
    if(i2c.read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) return false;

    /* Switch to config mode (just in case since this is the default) */
    setMode(OPERATION_MODE_CONFIG);

    /* Reset */
    i2c.write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
    while (i2c.read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
    {
        // delay(10);
        waitFor(10, MILLI);
    }
    // delay(50);
    waitFor(50, MILLI);

    /* Set to normal power mode */
    i2c.write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    // delay(10);
    waitFor(10, MILLI);

    i2c.write8(BNO055_PAGE_ID_ADDR, 0);

    /* Set the output units */
    /*
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
    write8(BNO055_UNIT_SEL_ADDR, unitsel);
    */

    /* Configure axis mapping (see section 3.4) */
    /*
    write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
    delay(10);
    write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
    delay(10);
    */

    write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
    // delay(10);
    waitFor(10, MILLI);
    /* Set the requested operating mode (see section 3.3) */
    setMode(mode);
    // delay(20);
    waitFor(20, MILLI);

    return true;
}

void BNO055::setMode(adafruit_bno055_opmode_t mode)
{
    opmode = mode;
    i2c.write8(BNO055_OPR_MODE_ADDR, opmode);
    waitFor(30, MILLI);
}

void BNO055::setExtCrystalUse(boolean useExternal)
{
    adafruit_bno055_opmode_t modeback = opmode;

    /* Switch to config mode (just in case since this is the default) */
    setMode(OPERATION_MODE_CONFIG);
    waitFor(25, MILLI);
    i2c.write8(BNO055_PAGE_ID_ADDR, 0);
    if (useExternal)    i2c.write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
    else                i2c.write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
    waitFor(10, MILLI);
    /* Set the requested operating mode (see section 3.3) */
    setMode(modeback);
    waitFor(20, MILLI);
}

void BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
    i2c.write8(BNO055_PAGE_ID_ADDR, 0);

    /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

    if (system_status != 0)
        *system_status = i2c.read8(BNO055_SYS_STAT_ADDR);

    /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

    if (self_test_result != 0)
        *self_test_result = i2c.read8(BNO055_SELFTEST_RESULT_ADDR);

    /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operation mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

    if (system_error != 0)
        *system_error = i2c.read8(BNO055_SYS_ERR_ADDR);

    waitFor(200, MILLI);
}

void BNO055::getRevInfo(adafruit_bno055_rev_info_t* info)
{
    uint8_t a, b;

    memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

    /* Check the accelerometer revision */
    info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);

    /* Check the magnetometer revision */
    info->mag_rev   = read8(BNO055_MAG_REV_ID_ADDR);

    /* Check the gyroscope revision */
    info->gyro_rev  = read8(BNO055_GYRO_REV_ID_ADDR);

    /* Check the SW revision */
    info->bl_rev    = read8(BNO055_BL_REV_ID_ADDR);

    a = i2c.read8(BNO055_SW_REV_ID_LSB_ADDR);
    b = i2c.read8(BNO055_SW_REV_ID_MSB_ADDR);
    info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

uint8_t BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
    uint8_t calData = i2c.read8(BNO055_CALIB_STAT_ADDR);
    if (sys != NULL)    *sys = (calData >> 6) & 0x03;
    if (gyro != NULL)   *gyro = (calData >> 4) & 0x03;
    if (accel != NULL)  *accel = (calData >> 2) & 0x03;
    if (mag != NULL)    *mag = calData & 0x03;
    return calData;
}

int8_t BNO055::getTemp(void)
{
    return (int8_t) i2c.read8(BNO055_TEMP_ADDR);
}

imu::Vector<3> BNO055::getVector(adafruit_vector_type_t vector_type)
{
    imu::Vector<3> xyz;
    uint8_t buffer[6];
    memset(buffer, 0, 6);

    int16_t x, y, z;
    x = y = z = 0;

    /* Read vector data (6 bytes) */
    readLen((adafruit_bno055_reg_t) vector_type, buffer, 6);

    x = ((int16_t) buffer[0]) | (((int16_t) buffer[1]) << 8);
    y = ((int16_t) buffer[2]) | (((int16_t) buffer[3]) << 8);
    z = ((int16_t) buffer[4]) | (((int16_t) buffer[5]) << 8);

    /* Convert the value to an appropriate range (section 3.6.4) */
    /* and assign the value to the Vector type */
    switch(vector_type)
    {
        case VECTOR_MAGNETOMETER:
            /* 1uT = 16 LSB */
            xyz[0] = ((double)x)/16.0;
            xyz[1] = ((double)y)/16.0;
            xyz[2] = ((double)z)/16.0;
            break;
        case VECTOR_GYROSCOPE:
            /* 1dps = 16 LSB */
            xyz[0] = ((double)x)/16.0;
            xyz[1] = ((double)y)/16.0;
            xyz[2] = ((double)z)/16.0;
            break;
        case VECTOR_EULER:
            /* 1 degree = 16 LSB */
            xyz[0] = ((double)x)/16.0;
            xyz[1] = ((double)y)/16.0;
            xyz[2] = ((double)z)/16.0;
            break;
        case VECTOR_ACCELEROMETER:
        case VECTOR_LINEARACCEL:
        case VECTOR_GRAVITY:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x)/100.0;
            xyz[1] = ((double)y)/100.0;
            xyz[2] = ((double)z)/100.0;
        break;
    }

    return xyz;
}

imu::Quaternion BNO055::getQuat(void)
{
    uint8_t buffer[8];
    memset(buffer, 0, 8);

    int16_t x, y, z, w;
    x = y = z = w = 0;

    /* Read quat data (8 bytes) */
    readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    w = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);
    x = (((uint16_t) buffer[3]) << 8) | ((uint16_t) buffer[2]);
    y = (((uint16_t) buffer[5]) << 8) | ((uint16_t) buffer[4]);
    z = (((uint16_t) buffer[7]) << 8) | ((uint16_t) buffer[6]);

    /* Assign to Quaternion */
    /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
     3.6.5.5 Orientation (Quaternion)  */
    const double scale = (1.0 / (1<<14));
    imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
    return quat;
}

bool BNO055::getSensorOffsets(adafruit_bno055_offsets_t &offsets_type)
{
    if (isFullyCalibrated())
    {
        adafruit_bno055_opmode_t lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);
        waitFor(25, MILLI);

        offsets_type.accel_offset_x = (i2c.read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (i2c.read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (i2c.read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (i2c.read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (i2c.read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (i2c.read8(ACCEL_OFFSET_Z_LSB_ADDR));

        offsets_type.gyro_offset_x = (i2c.read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (i2c.read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y = (i2c.read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (i2c.read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z = (i2c.read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (i2c.read8(GYRO_OFFSET_Z_LSB_ADDR));

        offsets_type.mag_offset_x = (i2c.read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (i2c.read8(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y = (i2c.read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (i2c.read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z = (i2c.read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (i2c.read8(MAG_OFFSET_Z_LSB_ADDR));

        offsets_type.accel_radius = (i2c.read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (i2c.read8(ACCEL_RADIUS_LSB_ADDR));
        offsets_type.mag_radius = (i2c.read8(MAG_RADIUS_MSB_ADDR) << 8) | (i2c.read8(MAG_RADIUS_LSB_ADDR));

        setMode(lastMode);
        return true;
    }
    return false;
}

void BNO055::setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type)
{
    adafruit_bno055_opmode_t lastMode = opmode;
    setMode(OPERATION_MODE_CONFIG);
    waitFor(25, MILLI);

    i2c.write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
    i2c.write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
    i2c.write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
    i2c.write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
    i2c.write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
    i2c.write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

    i2c.write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
    i2c.write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
    i2c.write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
    i2c.write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
    i2c.write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
    i2c.write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

    i2c.write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
    i2c.write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
    i2c.write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
    i2c.write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
    i2c.write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
    i2c.write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

    i2c.write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
    i2c.write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

    i2c.write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
    i2c.write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

    setMode(lastMode);
}

bool BNO055::isFullyCalibrated(void)
{
    return getCalibration(0, 0, 0, 0) & 0xFF;
}

bool BNO055::readLen(adafruit_bno055_reg_t reg, byte * buffer, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        buffer[i] = i2c.read8(reg + i);
    }
    return true;
}
