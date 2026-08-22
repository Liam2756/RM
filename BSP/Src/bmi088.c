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
#define GYRO_CHIP_ID      0x00U
#define GYRO_SOFTRESET    0x14U
#define GYRO_RANGE        0x0FU
#define GYRO_BANDWIDTH    0x10U
#define GYRO_LPM1         0x11U
#define GYRO_X_L          0x02U
#define GYRO_CHIP_ID_VALUE 0x0FU

#define GYRO_SENSITIVITY  0.0010652644360316953f

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

void BMI088_Init(void)
{
    AccelCS(1U);
    GyroCS(1U);

    AccelWrite(ACC_SOFTRESET, 0xB6U);
    HAL_Delay(80U);
    AccelWrite(ACC_PWR_CONF, 0x00U);
    AccelWrite(ACC_PWR_CTRL, 0x04U);
    AccelWrite(ACC_CONF, 0xA8U);
    AccelWrite(ACC_RANGE, 0x00U);

    GyroWrite(GYRO_SOFTRESET, 0xB6U);
    HAL_Delay(80U);
    GyroWrite(GYRO_RANGE, 0x00U);
    GyroWrite(GYRO_BANDWIDTH, 0x02U);
    GyroWrite(GYRO_LPM1, 0x00U);
}

void BMI088_ReadGyro(float gyro[3])
{
    uint8_t data[8];
    int16_t raw;

    GyroReadBytes(GYRO_CHIP_ID, data, 8U);

    raw = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    gyro[0] = raw * GYRO_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    gyro[1] = raw * GYRO_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[7] << 8) | data[6]);
    gyro[2] = raw * GYRO_SENSITIVITY;
}
