/**
 ******************************************************************************
 * @file    motor_driver.c
 * @brief   DJI 电机反馈解析实现（设备驱动层）
 * @version 2.0
 * @date    2026-8-11
 ******************************************************************************
 */

#include <motor_driver.h>
#include <hero_motor_config.h>

#define MOTOR_ANGLE_H_OFFSET   0U
#define MOTOR_ANGLE_L_OFFSET   1U
#define MOTOR_SPEED_H_OFFSET   2U
#define MOTOR_SPEED_L_OFFSET   3U
#define MOTOR_CURRENT_H_OFFSET 4U
#define MOTOR_CURRENT_L_OFFSET 5U
#define MOTOR_TEMP_OFFSET      6U

static void MotorDriver_ParseFeedback(volatile Motor_Measure_t *motor,
                                      const uint8_t data[CAN_STD_DLC],
                                      uint8_t parse_flags)
{
    motor->rotor_mechanical_angle =
        ((uint16_t)data[MOTOR_ANGLE_H_OFFSET] << 8) |
        (uint16_t)data[MOTOR_ANGLE_L_OFFSET];
    motor->rotor_speed_rpm =
        (int16_t)(((uint16_t)data[MOTOR_SPEED_H_OFFSET] << 8) |
                  (uint16_t)data[MOTOR_SPEED_L_OFFSET]);
    motor->torque_current =
        (int16_t)(((uint16_t)data[MOTOR_CURRENT_H_OFFSET] << 8) |
                  (uint16_t)data[MOTOR_CURRENT_L_OFFSET]);

    if ((parse_flags & MOTOR_PARSE_FLAG_TEMPERATURE) != 0U)
    {
        motor->motor_temperature = data[MOTOR_TEMP_OFFSET];
    }

    motor->last_rx_tick = HAL_GetTick();
    motor->rx_count++;
    motor->has_feedback = 1U;
}

void MotorDriver_ParseCANFrame(CAN_HandleTypeDef *hcan,
                               uint32_t std_id,
                               const uint8_t data[CAN_STD_DLC])
{
    if (data == 0)
    {
        return;
    }

    if (hcan == &hcan1)
    {
        switch (std_id)
        {
            case CAN1_YAW_GM6020_ID:
                MotorDriver_ParseFeedback(&g_hero_gimbal_motor[1],
                                          data,
                                          MOTOR_6020_PARSE_FLAGS);
                break;
            case CAN1_PITCH_GM6020_ID:
                MotorDriver_ParseFeedback(&g_hero_gimbal_motor[0],
                                          data,
                                          MOTOR_6020_PARSE_FLAGS);
                break;
            default:
                break;
        }
    }
}

uint8_t MotorDriver_IsOnline(const volatile Motor_Measure_t *motor,
                             uint32_t now_tick,
                             uint32_t timeout_ms)
{
    if ((motor == 0) || (motor->has_feedback == 0U))
    {
        return 0U;
    }
    return ((now_tick - motor->last_rx_tick) <= timeout_ms) ? 1U : 0U;
}

void MotorDriver_GetSnapshot(const volatile Motor_Measure_t *motor,
                             Motor_Measure_t *snapshot)
{
    uint32_t primask;

    if ((motor == 0) || (snapshot == 0))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->rotor_mechanical_angle = motor->rotor_mechanical_angle;
    snapshot->rotor_speed_rpm = motor->rotor_speed_rpm;
    snapshot->torque_current = motor->torque_current;
    snapshot->motor_temperature = motor->motor_temperature;
    snapshot->has_feedback = motor->has_feedback;
    snapshot->last_rx_tick = motor->last_rx_tick;
    snapshot->rx_count = motor->rx_count;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void BSP_CAN_RxFrameCallback(CAN_HandleTypeDef *hcan,
                             uint32_t std_id,
                             const uint8_t data[CAN_STD_DLC])
{
    MotorDriver_ParseCANFrame(hcan, std_id, data);
}
