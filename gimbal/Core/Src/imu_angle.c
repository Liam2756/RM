#include "imu_angle.h"

void IMU_Angle_Init(IMU_Angle_t *angle)
{
    angle->roll = 0.0f;
    angle->pitch = 0.0f;
    angle->yaw = 0.0f;
}

void IMU_Angle_Update(IMU_Angle_t *angle, const float gyro[3], float dt)
{
    angle->roll += gyro[0] * dt;
    angle->pitch += gyro[1] * dt;
    angle->yaw += gyro[2] * dt;
}
