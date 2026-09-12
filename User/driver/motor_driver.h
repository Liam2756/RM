/**
 ******************************************************************************
 * @file    motor_driver.h
 * @brief   DJI 电机反馈解析接口（设备驱动层）
 * @note    按机器人 CAN 拓扑进行固定分发，结构与参考工程一致。
 * @version 2.0
 * @date    2026-8-11
 ******************************************************************************
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <bsp_can.h>
#include <stdint.h>

#define MOTOR_PARSE_FLAG_TEMPERATURE 0x01U
#define MOTOR_3508_PARSE_FLAGS MOTOR_PARSE_FLAG_TEMPERATURE
#define MOTOR_6020_PARSE_FLAGS MOTOR_PARSE_FLAG_TEMPERATURE

typedef struct
{
    uint16_t rotor_mechanical_angle;
    int16_t rotor_speed_rpm;
    int16_t torque_current;
    uint8_t motor_temperature;
    uint8_t has_feedback;
    uint32_t last_rx_tick;
    uint32_t rx_count;
} Motor_Measure_t;

/** 解析一帧完整的 8 字节标准数据帧，公开以便台架测试。 */
void MotorDriver_ParseCANFrame(CAN_HandleTypeDef *hcan,
                               uint32_t std_id,
                               const uint8_t data[CAN_STD_DLC]);

/** 根据是否收到过反馈及最后接收时间判断在线状态。 */
uint8_t MotorDriver_IsOnline(const volatile Motor_Measure_t *motor,
                             uint32_t now_tick,
                             uint32_t timeout_ms);

/** 在短临界区内取得一致的电机反馈快照。 */
void MotorDriver_GetSnapshot(const volatile Motor_Measure_t *motor,
                             Motor_Measure_t *snapshot);

#endif /* MOTOR_DRIVER_H */
