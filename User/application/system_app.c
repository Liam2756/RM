#include "system_app.h"

#include "app.h"
#include "bmi088.h"
#include "bsp_uart.h"
#include "crc.h"
#include "imu_temperature.h"
#include "tim.h"
#include "uart_ports.h"
#include "usart.h"

#include <stdint.h>
#include <string.h>

#define IMU_UART_FRAME_HEADER     (0x5A)
#define IMU_UART_FRAME_SIZE       ((uint16_t)(1U + 4U * sizeof(float) + sizeof(uint32_t) + 2U))

IMU_Attitude_t imu_attitude;
float imu_update_dt_s;

static uint8_t imu_frame[IMU_UART_FRAME_SIZE];

static void IMU_BuildFrame(uint8_t *frame, const IMU_Attitude_t *attitude,
                           uint32_t timestamp_us)
{
    frame[0] = IMU_UART_FRAME_HEADER;
    memcpy(&frame[1], &attitude->q0, sizeof(float));
    memcpy(&frame[1 + sizeof(float)], &attitude->q1, sizeof(float));
    memcpy(&frame[1 + 2 * sizeof(float)], &attitude->q2, sizeof(float));
    memcpy(&frame[1 + 3 * sizeof(float)], &attitude->q3, sizeof(float));
    memcpy(&frame[1 + 4 * sizeof(float)], &timestamp_us, sizeof(timestamp_us));
    CRC16_Append(frame, IMU_UART_FRAME_SIZE);
}

void User_System_Init(void)
{
    if (!BMI088_Init())
    {
        Error_Handler();
    }

    IMU_Temperature_Init();
    UART_Ports_Init();
    IMU_Attitude_Init(&imu_attitude);
    imu_update_dt_s = (float)((htim1.Init.Prescaler + 1U) *
                              (htim1.Init.Period + 1U)) /
                      (float)SystemCoreClock;
    App_Init();
}

void User_System_Tick1ms(void)
{
    uint32_t timestamp_us = __HAL_TIM_GET_COUNTER(&htim2);

    if (IMU_Attitude_Update(&imu_attitude, imu_update_dt_s))
    {
        IMU_BuildFrame(imu_frame, &imu_attitude, timestamp_us);
        (void)BSP_UART_TxData(&huart6, imu_frame, IMU_UART_FRAME_SIZE);
    }

    IMU_Temperature_Tick1ms();
    App_ControlTask1ms();
}
