#ifndef IMU_ANGLE_H
#define IMU_ANGLE_H

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} IMU_Angle_t;

void IMU_Angle_Init(IMU_Angle_t *angle);
void IMU_Angle_Update(IMU_Angle_t *angle, const float gyro[3], float dt);

#endif
