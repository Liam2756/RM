#include "imu_attitude.h"

#include <math.h>

static void IMU_Attitude_UpdateEuler(IMU_Attitude_t *attitude)
{
    float pitch_sine = 2.0f * (attitude->q0 * attitude->q2 - attitude->q3 * attitude->q1);
    if (pitch_sine > 1.0f)
        pitch_sine = 1.0f;
    else if (pitch_sine < -1.0f)
        pitch_sine = -1.0f;

    attitude->roll = atan2f(2.0f * (attitude->q0 * attitude->q1 + attitude->q2 * attitude->q3),
                            1.0f - 2.0f * (attitude->q1 * attitude->q1 + attitude->q2 * attitude->q2));
    attitude->pitch = asinf(pitch_sine);
    attitude->yaw = atan2f(2.0f * (attitude->q0 * attitude->q3 + attitude->q1 * attitude->q2),
                           1.0f - 2.0f * (attitude->q2 * attitude->q2 + attitude->q3 * attitude->q3));
}

void IMU_Attitude_Init(IMU_Attitude_t *attitude, const float accel[3])
{
    const float accel_horizontal = sqrtf(accel[1] * accel[1] + accel[2] * accel[2]);

    attitude->roll = atan2f(accel[1], accel[2]);
    attitude->pitch = atan2f(-accel[0], accel_horizontal);
    attitude->yaw = 0.0f;

    const float half_roll = 0.5f * attitude->roll;
    const float half_pitch = 0.5f * attitude->pitch;
    const float half_yaw = 0.0f;

    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);
    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);

    attitude->q0 = cr * cp * cy + sr * sp * sy;
    attitude->q1 = sr * cp * cy - cr * sp * sy;
    attitude->q2 = cr * sp * cy + sr * cp * sy;
    attitude->q3 = cr * cp * sy - sr * sp * cy;
    attitude->integral_error[0] = 0.0f;
    attitude->integral_error[1] = 0.0f;
    attitude->integral_error[2] = 0.0f;
}

void IMU_Attitude_Update(IMU_Attitude_t *attitude,
                         const float gyro[3],
                         const float accel[3],
                         float dt)
{
    float gx = gyro[0];
    float gy = gyro[1];
    float gz = gyro[2];
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
    IMU_Attitude_UpdateEuler(attitude);
}
