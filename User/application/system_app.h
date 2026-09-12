#ifndef SYSTEM_APP_H
#define SYSTEM_APP_H

#include "imu_attitude.h"

extern IMU_Attitude_t imu_attitude;
extern float imu_update_dt_s;

void User_System_Init(void);
void User_System_Tick1ms(void);

#endif /* SYSTEM_APP_H */
