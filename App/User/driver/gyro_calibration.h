/**
 ******************************************************************************
 * @file    gyro_calibration.h
 * @brief   陀螺仪零偏补偿模块头文件（设备驱动层）
 * @note    层级说明：
 *            本文件属于【设备驱动层】，位于 BSP 层之上、算法层/控制层之下。
 *            原因：本模块通过原 IMU 数据接口获取传感器数据，
 *                  因此不归属于纯粹的"算法层"。
 *
 *          若未来需要支持其他 IMU，可将纯算法部分（is_static / 偏置 IIR）
 *          提取为独立的算法层模块，gyro_calibration 只负责读取数据并调用它。
 *
 *          使用流程（顺序不能颠倒）：
 *
 *            // 1. 初始化阶段（在 HAL_TIM_Base_Start_IT 之前，机器人须静止）
 *            GyroBias_t gyro_bias;
 *            GyroBias_StaticCalib(&gyro_bias);   // 阻塞约 2s
 *
 *            // 2. 每帧（在 IMU_Attitude_ReadSensor 之后立即调用）
 *            IMU_Attitude_ReadSensor(gyro, accel);
 *            GyroBias_Correct(&gyro_bias, gyro, accel);
 *            // gyro[] 已原地补偿，直接用于控制
 *
 * @version 1.0
 * @date    2026-4-4
 * @author  claude
 ******************************************************************************
 */

#ifndef GYRO_CALIBRATION_H
#define GYRO_CALIBRATION_H

#include <stdint.h>

/* ========================================================================== */
/*                              可调参数                                        */
/* ========================================================================== */

/** 静态标定采样帧数（@ 1kHz 调用 = 2 秒）*/
#define GYRO_CALIB_SAMPLE_NUM   2000U

/** 静止判断：补偿后陀螺仪任一轴幅值上限（rad/s，≈ 2.9°/s）*/
#define GYRO_STATIC_THRESHOLD   0.05f

/** 静止判断：加速度计幅值偏离标准重力的允许量（m/s²）*/
#define ACCEL_STATIC_THRESHOLD  0.5f

/**
 * @brief  静止确认帧数阈值
 * @note   需连续多少帧满足静止条件才触发动态偏置更新，
 *         防止短暂停顿时误更新。
 */
#define STATIC_CONFIRM_COUNT    200U

/**
 * @brief  动态偏置更新 IIR 系数
 * @note   越小越保守（响应慢但抗运动误判），推荐范围 [0.0002, 0.001]。
 *         bias += alpha * corrected_gyro（静止时补偿后残余即为新偏置分量）
 */
#define BIAS_UPDATE_ALPHA       0.0005f

/** 标准重力加速度（m/s²）*/
#define GRAVITY_MSS             9.80665f

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  陀螺仪零偏状态结构体
 */
typedef struct
{
    float    bias[3];         /**< 当前零偏估计，三轴，单位 rad/s       */
    uint8_t  calibrated;      /**< 静态标定完成标志，1=已完成           */
    uint16_t static_counter;  /**< 连续静止帧计数，用于动态更新确认     */
} GyroBias_t;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  上电静态零偏标定（阻塞式）
 * @note   阻塞约 GYRO_CALIB_SAMPLE_NUM ms，期间机器人必须保持水平静止。
 *         内部对加速度计数据做有效性筛选，剔除搬运过程中的脏帧。
 *         有效帧数 < 50% 时，bias 保持 0 不写入（相对更安全）。
 *         完成后 gb->calibrated = 1。
 * @param  gb  零偏状态结构体指针（由调用方持有，函数内部 memset 清零）
 * @retval 无
 */
void GyroBias_StaticCalib(GyroBias_t *gb);

/**
 * @brief  每帧补偿 + 动态零偏更新（原地修改 gyro[]）
 * @note   执行顺序：
 *           1. 用当前 bias 对 gyro[] 原地补偿
 *           2. 用补偿后值 + accel[] 判断是否静止
 *           3. 连续静止 STATIC_CONFIRM_COUNT 帧后，用 IIR 缓慢修正 bias
 *         调用后 gyro[] 已是补偿值，直接用于下游控制。
 * @param  gb     零偏状态结构体指针
 * @param  gyro   IMU 输出的陀螺仪原始数据（rad/s），原地修改
 * @param  accel  IMU 输出的加速度计数据（m/s²），只读
 * @retval 无
 */
void GyroBias_Correct(GyroBias_t *gb, float gyro[3], const float accel[3]);

#endif /* GYRO_CALIBRATION_H */
