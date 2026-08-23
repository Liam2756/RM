#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>

#include "main.h"

#define BMI088_ACCEL_RANGE_CONFIG      0x00U
#define BMI088_GYRO_RANGE_CONFIG       0x00U
#define BMI088_GYRO_BANDWIDTH_CONFIG   0x02U
#define BMI088_GYRO_SENSITIVITY        0.0010652644360316953f
#define BMI088_ACCEL_SENSITIVITY       0.0008974358974f

void BMI088_Init(void);
bool BMI088_Read(float gyro[3], float accel[3]);

#endif
