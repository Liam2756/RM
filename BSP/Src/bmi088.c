#include "bmi088.h"
#include "spi.h"

#define BMI088_ACCEL_CS_PORT GPIOA
#define BMI088_ACCEL_CS_PIN  GPIO_PIN_4
#define BMI088_GYRO_CS_PORT  GPIOB
#define BMI088_GYRO_CS_PIN   GPIO_PIN_0

#define ACC_CHIP_ID       0x00U
#define ACC_SOFTRESET     0x7EU
#define ACC_PWR_CONF      0x7CU
#define ACC_PWR_CTRL      0x7DU
#define ACC_CONF          0x40U
#define ACC_RANGE         0x41U
#define ACC_X_L           0x12U
#define GYRO_CHIP_ID      0x00U
#define GYRO_SOFTRESET    0x14U
#define GYRO_RANGE        0x0FU
#define GYRO_BANDWIDTH    0x10U
#define GYRO_LPM1         0x11U
#define GYRO_X_L          0x02U
#define GYRO_CHIP_ID_VALUE 0x0FU

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

static uint8_t SpiByte(uint8_t tx)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void AccelWrite(uint8_t reg, uint8_t value)
{
    AccelCS(0U);
    SpiByte(reg);
    SpiByte(value);
    AccelCS(1U);
}

static void GyroWrite(uint8_t reg, uint8_t value)
{
    GyroCS(0U);
    SpiByte(reg);
    SpiByte(value);
    GyroCS(1U);
}

static void GyroReadBytes(uint8_t reg, uint8_t *data, uint8_t length)
{
    GyroCS(0U);
    SpiByte(reg | 0x80U);
    while (length--)
        *data++ = SpiByte(0x55U);
    GyroCS(1U);
}

static void AccelReadBytes(uint8_t reg, uint8_t *data, uint8_t length)
{
    AccelCS(0U);
    SpiByte(reg | 0x80U);
    SpiByte(0x55U);
    while (length--)
        *data++ = SpiByte(0x55U);
    AccelCS(1U);
}

void BMI088_Init(void)
{
    AccelCS(1U);
    GyroCS(1U);

    AccelWrite(ACC_SOFTRESET, 0xB6U);
    HAL_Delay(80U);
    AccelWrite(ACC_PWR_CONF, 0x00U);
    AccelWrite(ACC_PWR_CTRL, 0x04U);
    AccelWrite(ACC_CONF, 0xA8U);
    AccelWrite(ACC_RANGE, BMI088_ACCEL_RANGE_CONFIG);

    GyroWrite(GYRO_SOFTRESET, 0xB6U);
    HAL_Delay(80U);
    GyroWrite(GYRO_RANGE, BMI088_GYRO_RANGE_CONFIG);
    GyroWrite(GYRO_BANDWIDTH, BMI088_GYRO_BANDWIDTH_CONFIG);
    GyroWrite(GYRO_LPM1, 0x00U);
}

bool BMI088_Read(float gyro[3], float accel[3])
{
    uint8_t gyro_data[8];
    uint8_t accel_data[6];
    int16_t raw;

    AccelReadBytes(ACC_X_L, accel_data, 6U);
    GyroReadBytes(GYRO_CHIP_ID, gyro_data, 8U);

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
