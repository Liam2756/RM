#ifndef IMU_TEMPERATURE_H
#define IMU_TEMPERATURE_H

#include <stdint.h>

#define IMU_TEMP_TARGET_C          45.0f
#define IMU_TEMP_OVERHEAT_C        50.0f
#define IMU_TEMP_CONTROL_TICKS     1280U
#define IMU_TEMP_PWM_FULL_SCALE    5000U

#define IMU_TEMP_KP                1000.0f
#define IMU_TEMP_KI                20.0f
#define IMU_TEMP_KD                0.0f
#define IMU_TEMP_I_MAX             5000.0f
#define IMU_TEMP_I_GAP_C           5.0f

typedef struct
{
    volatile float temperature_celsius;
    volatile uint16_t pwm_pulse;
    volatile uint8_t overtemperature;
    volatile uint32_t read_ok_count;
    volatile uint32_t read_fail_count;
} IMU_Temperature_Stats_t;

extern IMU_Temperature_Stats_t imu_temperature_stats;

void IMU_Temperature_Init(void);
void IMU_Temperature_Tick1ms(void);

#endif
