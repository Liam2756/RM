/**
 ******************************************************************************
 * @file    gyro_calibration.c
 * @brief   陀螺仪零偏补偿实现（设备驱动层）
 * @note    职责边界：
 *            - 调用原 IMU 数据接口采集原始数据
 *            - 执行静态标定与动态 IIR 零偏估计
 *          【禁止】在本文件中调用控制层 / 应用层接口
 * @version 1.0
 * @date    2026-4-4
 * @author  claude
 ******************************************************************************
 */

#include "gyro_calibration.h"
#include "imu_attitude.h"        /* IMU_Attitude_ReadSensor() */
#include "main.h"
#include <math.h>               /* fabsf, sqrtf       */
#include <string.h>             /* memset             */

/* ========================================================================== */
/*                              内部函数                                        */
/* ========================================================================== */

/**
 * @brief  综合静止检测
 * @note   双条件同时满足才认为静止：
 *           条件 A：补偿后陀螺仪三轴幅值均 < GYRO_STATIC_THRESHOLD
 *           条件 B：加速度计幅值偏离标准重力 < ACCEL_STATIC_THRESHOLD
 *         条件 B 可排除底盘平移加速、云台旋转产生的切向加速度导致的误判，
 *         仅用陀螺仪判断时对低速运动的排斥能力不足。
 * @param  gyro_corrected  补偿后陀螺仪数据（rad/s）
 * @param  accel           加速度计数据（m/s²）
 * @retval 1: 静止, 0: 运动中
 */
static uint8_t IsStatic(const float gyro_corrected[3], const float accel[3])
{
    /* 条件 A：陀螺仪任一轴超阈值即为运动 */
    if (fabsf(gyro_corrected[0]) > GYRO_STATIC_THRESHOLD ||
        fabsf(gyro_corrected[1]) > GYRO_STATIC_THRESHOLD ||
        fabsf(gyro_corrected[2]) > GYRO_STATIC_THRESHOLD)
    {
        return 0U;
    }

    /* 条件 B：加速度计合量偏离重力加速度 */
    float accel_mag = sqrtf(accel[0] * accel[0] +
                            accel[1] * accel[1] +
                            accel[2] * accel[2]);

    if (fabsf(accel_mag - GRAVITY_MSS) > ACCEL_STATIC_THRESHOLD)
    {
        return 0U;
    }

    return 1U;
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  上电静态零偏标定（阻塞式）
 */
void GyroBias_StaticCalib(GyroBias_t *gb)
{
    memset(gb, 0, sizeof(GyroBias_t));

    float   gyro[3], accel[3];
    double  sum[3]       = {0.0, 0.0, 0.0}; /* double 累加减少浮点精度损失 */
    uint32_t valid_count = 0U;

    for (uint32_t i = 0U; i < GYRO_CALIB_SAMPLE_NUM; i++)
    {
        IMU_Attitude_ReadSensor(gyro, accel);

        /* 筛选帧：加速度计幅值正常才计入均值，剔除振动/搬运脏帧 */
        float accel_mag = sqrtf(accel[0] * accel[0] +
                                accel[1] * accel[1] +
                                accel[2] * accel[2]);

        if (fabsf(accel_mag - GRAVITY_MSS) < ACCEL_STATIC_THRESHOLD)
        {
            sum[0] += (double)gyro[0];
            sum[1] += (double)gyro[1];
            sum[2] += (double)gyro[2];
            valid_count++;
        }

        HAL_Delay(1U); /* 1ms/帧，与定时器中断频率对齐 */
    }

    /*
     * 有效帧数不足 50% 说明标定期间存在明显干扰（如被搬运）。
     * 此时保持 bias = 0，避免用少量脏数据引入错误偏置。
     */
    if (valid_count >= GYRO_CALIB_SAMPLE_NUM / 2U)
    {
        gb->bias[0] = (float)(sum[0] / (double)valid_count);
        gb->bias[1] = (float)(sum[1] / (double)valid_count);
        gb->bias[2] = (float)(sum[2] / (double)valid_count);
    }

    gb->calibrated    = 1U;
    gb->static_counter = 0U;
}

/**
 * @brief  每帧补偿 + 动态零偏更新
 */
void GyroBias_Correct(GyroBias_t *gb, float gyro[3], const float accel[3])
{
    /* Step 1：对原始数据施加当前零偏补偿 */
    float corrected[3] = {
        gyro[0] - gb->bias[0],
        gyro[1] - gb->bias[1],
        gyro[2] - gb->bias[2],
    };

    /* Step 2：静止检测，累计连续静止帧数 */
    if (IsStatic(corrected, accel))
    {
        gb->static_counter++;
    }
    else
    {
        gb->static_counter = 0U; /* 运动中立即清零，防止短暂停顿触发更新 */
    }

    /* Step 3：连续静止足够帧后，用 IIR 缓慢修正零偏估计
     *         原理：静止时补偿后残余输出 = 温漂/时漂产生的新偏置分量
     *         bias += alpha * corrected 将其缓慢纳入估计
     */
    if (gb->static_counter >= STATIC_CONFIRM_COUNT)
    {
        gb->bias[0] += BIAS_UPDATE_ALPHA * corrected[0];
        gb->bias[1] += BIAS_UPDATE_ALPHA * corrected[1];
        gb->bias[2] += BIAS_UPDATE_ALPHA * corrected[2];
    }

    /* Step 4：将补偿后的值写回 gyro[]（调用方直接使用，无需额外变量）*/
    gyro[0] = corrected[0];
    gyro[1] = corrected[1];
    gyro[2] = corrected[2];
}
