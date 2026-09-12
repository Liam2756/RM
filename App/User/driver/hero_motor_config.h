/**
 ******************************************************************************
 * @file    hero_motor_config.h
 * @brief   英雄机器人电机拓扑配置接口
 * @note    只保存本机器人专属的数量、总线、反馈 ID 和控制帧槽位。
 * @version 1.0
 * @date    2026-8-11
 ******************************************************************************
 */

#ifndef HERO_MOTOR_CONFIG_H
#define HERO_MOTOR_CONFIG_H

#include <motor_driver.h>
#include <motor_tx.h>

#define HERO_GIMBAL_MOTOR_COUNT   2U

extern volatile Motor_Measure_t g_hero_gimbal_motor[HERO_GIMBAL_MOTOR_COUNT];

HAL_StatusTypeDef HeroMotorConfig_Init(void);

/** @brief 云台 GM6020 电流控制量（motor_index：0=Pitch/CAN1/ID2，1=Yaw/CAN1/ID1）。 */
HAL_StatusTypeDef HeroMotorTx_SetGimbalCurrent(uint8_t motor_index,
                                               int16_t current);
void HeroMotorTx_ClearAll(void);
HAL_StatusTypeDef HeroMotorTx_FlushAll(void);

#endif /* HERO_MOTOR_CONFIG_H */
