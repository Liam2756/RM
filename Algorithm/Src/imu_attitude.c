#include "imu_attitude.h"

#include "bmi088.h"
#include "imu_config.h"

#include <math.h>

void IMU_Attitude_Init(IMU_Attitude_t *attitude)
{
    float gyro[3];
    float accel[3];
    float accel_avg[3] = {0.0f, 0.0f, 0.0f};
    double gyro_sum[3] = {0.0, 0.0, 0.0};
    double accel_sum[3] = {0.0, 0.0, 0.0};
    uint32_t valid_samples = 0U;

    for (uint32_t sample = 0U; sample < IMU_CALIBRATION_SAMPLE_COUNT; sample++)
    {
        if (BMI088_Read(gyro, accel))
        {
            const float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
            if (fabsf(accel_norm - IMU_ATTITUDE_GRAVITY_MSS) <= IMU_CALIBRATION_ACCEL_TOLERANCE_MSS)
            {
                for (uint32_t axis = 0U; axis < 3U; axis++)
                {
                    gyro_sum[axis] += gyro[axis];
                    accel_sum[axis] += accel[axis];
                }
                valid_samples++;
            }
        }
        HAL_Delay(1U);
    }

    if (valid_samples > 0U)
    {
        for (uint32_t axis = 0U; axis < 3U; axis++)
        {
            attitude->gyro_bias[axis] = (float)(gyro_sum[axis] / valid_samples);
            accel_avg[axis] = (float)(accel_sum[axis] / valid_samples);
        }
    }

    const float accel_norm = sqrtf(accel_avg[0] * accel_avg[0] + accel_avg[1] * accel_avg[1] + accel_avg[2] * accel_avg[2]);

    if (accel_norm < 0.1f)
    {
        attitude->q0 = 1.0f;
        attitude->q1 = 0.0f;
        attitude->q2 = 0.0f;
        attitude->q3 = 0.0f;
    }
    else
    {
        const float ax = accel_avg[0] / accel_norm;
        const float ay = accel_avg[1] / accel_norm;
        const float az = accel_avg[2] / accel_norm;
        const float horizontal = sqrtf(ay * ay + az * az);

        if (horizontal < 1e-6f)
        {
            const float q0 = sqrtf(0.5f);
            attitude->q0 = q0;
            attitude->q1 = 0.0f;
            attitude->q2 = copysignf(q0, -ax);
            attitude->q3 = 0.0f;
        }
        else
        {
            const float half_roll_cos = sqrtf((horizontal + az) / (2.0f * horizontal));
            const float half_roll_sine = copysignf(sqrtf((horizontal - az) / (2.0f * horizontal)), ay);
            const float half_pitch_cos = sqrtf(0.5f * (1.0f + horizontal));
            const float half_pitch_sine = copysignf(sqrtf(0.5f * (1.0f - horizontal)), -ax);

            attitude->q0 = half_roll_cos * half_pitch_cos;
            attitude->q1 = half_roll_sine * half_pitch_cos;
            attitude->q2 = half_roll_cos * half_pitch_sine;
            attitude->q3 = 0.0f;
        }
    }

    attitude->integral_error[0] = 0.0f;
    attitude->integral_error[1] = 0.0f;
    attitude->integral_error[2] = 0.0f;
}

bool IMU_Attitude_Update(IMU_Attitude_t *attitude, float dt)
{
    float gyro[3];
    float accel[3];

    if (!BMI088_Read(gyro, accel))
        return false;

    float gx = gyro[0] - attitude->gyro_bias[0];
    float gy = gyro[1] - attitude->gyro_bias[1];
    float gz = gyro[2] - attitude->gyro_bias[2];
    const float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

#if IMU_ATTITUDE_USE_ACCEL_CORRECTION
    if (fabsf(accel_norm - IMU_ATTITUDE_GRAVITY_MSS) <= IMU_ATTITUDE_ACCEL_TRUST_TOLERANCE_MSS)
    {
        const float ax = accel[0] / accel_norm;
        const float ay = accel[1] / accel_norm;
        const float az = accel[2] / accel_norm;
        const float half_vx = attitude->q1 * attitude->q3 - attitude->q0 * attitude->q2;
        const float half_vy = attitude->q0 * attitude->q1 + attitude->q2 * attitude->q3;
        const float half_vz = attitude->q0 * attitude->q0 - 0.5f + attitude->q3 * attitude->q3;
        const float half_ex = ay * half_vz - az * half_vy;
        const float half_ey = az * half_vx - ax * half_vz;
        const float half_ez = ax * half_vy - ay * half_vx;

        attitude->integral_error[0] += IMU_ATTITUDE_MAHONY_KI * half_ex * dt;
        attitude->integral_error[1] += IMU_ATTITUDE_MAHONY_KI * half_ey * dt;
        attitude->integral_error[2] += IMU_ATTITUDE_MAHONY_KI * half_ez * dt;

        gx += IMU_ATTITUDE_MAHONY_KP * half_ex + attitude->integral_error[0];
        gy += IMU_ATTITUDE_MAHONY_KP * half_ey + attitude->integral_error[1];
        gz += IMU_ATTITUDE_MAHONY_KP * half_ez + attitude->integral_error[2];
    }
#else
    (void)accel_norm;
#endif

    const float half_dt = 0.5f * dt;
    const float q0 = attitude->q0;
    const float q1 = attitude->q1;
    const float q2 = attitude->q2;
    const float q3 = attitude->q3;

    attitude->q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    attitude->q1 += ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    attitude->q2 += ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    attitude->q3 += ( q0 * gz + q1 * gy - q2 * gx) * half_dt;

    const float quaternion_norm = sqrtf(attitude->q0 * attitude->q0 +
                                        attitude->q1 * attitude->q1 +
                                        attitude->q2 * attitude->q2 +
                                        attitude->q3 * attitude->q3);
    attitude->q0 /= quaternion_norm;
    attitude->q1 /= quaternion_norm;
    attitude->q2 /= quaternion_norm;
    attitude->q3 /= quaternion_norm;

    return true;
}
