#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#define IMU_ATTITUDE_USE_ACCEL_CORRECTION  1U
#define IMU_ATTITUDE_MAHONY_KP             2.0f
#define IMU_ATTITUDE_MAHONY_KI             0.05f
#define IMU_ATTITUDE_GRAVITY_MSS           9.80665f
#define IMU_ATTITUDE_ACCEL_TRUST_TOLERANCE_MSS  1.5f

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float integral_error[3];
    float roll;
    float pitch;
    float yaw;
} IMU_Attitude_t;

void IMU_Attitude_Init(IMU_Attitude_t *attitude, const float accel[3]);
void IMU_Attitude_Update(IMU_Attitude_t *attitude,
                         const float gyro[3],
                         const float accel[3],
                         float dt);

#endif
