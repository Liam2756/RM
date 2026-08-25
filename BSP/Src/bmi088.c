#include "bmi088.h"
#include "spi.h"

#define BMI088_SPI_TIMEOUT_MS 10U

#define BMI088_ACCEL_CS_PORT GPIOA
#define BMI088_ACCEL_CS_PIN  GPIO_PIN_4
#define BMI088_GYRO_CS_PORT  GPIOB
#define BMI088_GYRO_CS_PIN   GPIO_PIN_0

#define ACC_CHIP_ID          0x00U
#define ACC_CHIP_ID_VALUE    0x1EU
#define ACC_SOFTRESET        0x7EU
#define ACC_PWR_CONF         0x7CU
#define ACC_PWR_CTRL         0x7DU
#define ACC_CONF             0x40U
#define ACC_RANGE            0x41U
#define ACC_INT1_IO_CTRL     0x53U
#define ACC_INT_MAP_DATA     0x58U
#define ACC_X_L              0x12U

#define GYRO_CHIP_ID         0x00U
#define GYRO_CHIP_ID_VALUE   0x0FU
#define GYRO_SOFTRESET       0x14U
#define GYRO_RANGE           0x0FU
#define GYRO_BANDWIDTH       0x10U
#define GYRO_LPM1            0x11U
#define GYRO_CTRL            0x15U
#define GYRO_INT3_INT4_CONF  0x16U
#define GYRO_INT3_INT4_MAP   0x18U

#define ACC_PWR_ACTIVE       0x00U
#define ACC_ENABLE           0x04U
#define ACC_CONF_VALUE       0xABU
#define ACC_RANGE_VALUE      0x00U
#define ACC_INT1_CONF_VALUE  0x08U
#define ACC_INT_MAP_VALUE    0x04U

#define GYRO_RANGE_VALUE     0x00U
#define GYRO_BANDWIDTH_VALUE BMI088_GYRO_BANDWIDTH_CONFIG
#define GYRO_NORMAL_MODE     0x00U
#define GYRO_DRDY_ON         0x80U
#define GYRO_INT_CONF_VALUE  0x00U
#define GYRO_INT_MAP_VALUE   0x01U

static void AccelCS(uint8_t level)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void GyroCS(uint8_t level)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool SpiByte(uint8_t tx, uint8_t *rx)
{
    return HAL_SPI_TransmitReceive(&hspi1, &tx, rx, 1U,
                                   BMI088_SPI_TIMEOUT_MS) == HAL_OK;
}

static void BMI088_ConfigDelay(void)
{
    HAL_Delay(1U);
}

static bool AccelWrite(uint8_t reg, uint8_t value)
{
    uint8_t rx;
    bool ok;

    AccelCS(0U);
    ok = SpiByte(reg, &rx);
    if (ok)
        ok = SpiByte(value, &rx);
    AccelCS(1U);
    return ok;
}

static bool GyroWrite(uint8_t reg, uint8_t value)
{
    uint8_t rx;
    bool ok;

    GyroCS(0U);
    ok = SpiByte(reg, &rx);
    if (ok)
        ok = SpiByte(value, &rx);
    GyroCS(1U);
    return ok;
}

static bool AccelReadBytes(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t rx;
    bool ok = true;

    AccelCS(0U);
    ok = SpiByte(reg | 0x80U, &rx);
    if (ok)
        ok = SpiByte(0x55U, &rx);
    for (uint8_t index = 0U; ok && index < length; index++)
        ok = SpiByte(0x55U, &data[index]);
    AccelCS(1U);
    return ok;
}

static bool GyroReadBytes(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t rx;
    bool ok = true;

    GyroCS(0U);
    ok = SpiByte(reg | 0x80U, &rx);
    for (uint8_t index = 0U; ok && index < length; index++)
        ok = SpiByte(0x55U, &data[index]);
    GyroCS(1U);
    return ok;
}

static bool AccelReadReg(uint8_t reg, uint8_t *value)
{
    return AccelReadBytes(reg, value, 1U);
}

static bool GyroReadReg(uint8_t reg, uint8_t *value)
{
    return GyroReadBytes(reg, value, 1U);
}

static bool AccelWriteAndVerify(uint8_t reg, uint8_t value)
{
    uint8_t actual;

    if (!AccelWrite(reg, value))
        return false;
    BMI088_ConfigDelay();
    return AccelReadReg(reg, &actual) && actual == value;
}

static bool GyroWriteAndVerify(uint8_t reg, uint8_t value)
{
    uint8_t actual;

    if (!GyroWrite(reg, value))
        return false;
    BMI088_ConfigDelay();
    return GyroReadReg(reg, &actual) && actual == value;
}

static bool BMI088_InitAccel(void)
{
    uint8_t chip_id;

    (void)AccelReadReg(ACC_CHIP_ID, &chip_id);
    BMI088_ConfigDelay();
    (void)AccelReadReg(ACC_CHIP_ID, &chip_id);
    BMI088_ConfigDelay();

    if (!AccelWrite(ACC_SOFTRESET, 0xB6U))
        return false;
    HAL_Delay(80U);

    if (!AccelReadReg(ACC_CHIP_ID, &chip_id))
        return false;
    BMI088_ConfigDelay();
    if (!AccelReadReg(ACC_CHIP_ID, &chip_id) || chip_id != ACC_CHIP_ID_VALUE)
        return false;

    if (!AccelWriteAndVerify(ACC_PWR_CTRL, ACC_ENABLE))
        return false;
    if (!AccelWriteAndVerify(ACC_PWR_CONF, ACC_PWR_ACTIVE))
        return false;
    if (!AccelWriteAndVerify(ACC_CONF, ACC_CONF_VALUE))
        return false;
    if (!AccelWriteAndVerify(ACC_RANGE, ACC_RANGE_VALUE))
        return false;
    if (!AccelWriteAndVerify(ACC_INT1_IO_CTRL, ACC_INT1_CONF_VALUE))
        return false;
    return AccelWriteAndVerify(ACC_INT_MAP_DATA, ACC_INT_MAP_VALUE);
}

static bool BMI088_InitGyro(void)
{
    uint8_t chip_id;

    (void)GyroReadReg(GYRO_CHIP_ID, &chip_id);
    BMI088_ConfigDelay();
    (void)GyroReadReg(GYRO_CHIP_ID, &chip_id);
    BMI088_ConfigDelay();

    if (!GyroWrite(GYRO_SOFTRESET, 0xB6U))
        return false;
    HAL_Delay(80U);

    if (!GyroReadReg(GYRO_CHIP_ID, &chip_id))
        return false;
    BMI088_ConfigDelay();
    if (!GyroReadReg(GYRO_CHIP_ID, &chip_id) || chip_id != GYRO_CHIP_ID_VALUE)
        return false;

    if (!GyroWriteAndVerify(GYRO_RANGE, GYRO_RANGE_VALUE))
        return false;
    if (!GyroWriteAndVerify(GYRO_BANDWIDTH, GYRO_BANDWIDTH_VALUE))
        return false;
    if (!GyroWriteAndVerify(GYRO_LPM1, GYRO_NORMAL_MODE))
        return false;
    if (!GyroWriteAndVerify(GYRO_CTRL, GYRO_DRDY_ON))
        return false;
    if (!GyroWriteAndVerify(GYRO_INT3_INT4_CONF, GYRO_INT_CONF_VALUE))
        return false;
    return GyroWriteAndVerify(GYRO_INT3_INT4_MAP, GYRO_INT_MAP_VALUE);
}

bool BMI088_Init(void)
{
    AccelCS(1U);
    GyroCS(1U);

    return BMI088_InitAccel() && BMI088_InitGyro();
}

bool BMI088_Read(float gyro[3], float accel[3])
{
    uint8_t gyro_data[8];
    uint8_t accel_data[6];
    int16_t raw;

    if ((gyro == 0) || (accel == 0))
        return false;

    if (!AccelReadBytes(ACC_X_L, accel_data, 6U) ||
        !GyroReadBytes(GYRO_CHIP_ID, gyro_data, 8U))
        return false;

    if (gyro_data[0] != GYRO_CHIP_ID_VALUE)
        return false;

    raw = (int16_t)(((uint16_t)gyro_data[3] << 8) | gyro_data[2]);
    gyro[0] = raw * BMI088_GYRO_SENSITIVITY;
    raw = (int16_t)(((uint16_t)gyro_data[5] << 8) | gyro_data[4]);
    gyro[1] = raw * BMI088_GYRO_SENSITIVITY;
    raw = (int16_t)(((uint16_t)gyro_data[7] << 8) | gyro_data[6]);
    gyro[2] = raw * BMI088_GYRO_SENSITIVITY;

    raw = (int16_t)(((uint16_t)accel_data[1] << 8) | accel_data[0]);
    accel[0] = raw * BMI088_ACCEL_SENSITIVITY;
    raw = (int16_t)(((uint16_t)accel_data[3] << 8) | accel_data[2]);
    accel[1] = raw * BMI088_ACCEL_SENSITIVITY;
    raw = (int16_t)(((uint16_t)accel_data[5] << 8) | accel_data[4]);
    accel[2] = raw * BMI088_ACCEL_SENSITIVITY;

    return true;
}
