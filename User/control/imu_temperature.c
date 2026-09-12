#include "imu_temperature.h"

#include "bmi088.h"
#include "bsp_pwm.h"
#include "pid.h"

#include <string.h>

IMU_Temperature_Stats_t imu_temperature_stats;

static const BSP_PWM_Channel_t heater_pwm = {&htim10, TIM_CHANNEL_1};
static const PID_Config_t temperature_pi_config = {
    .kp = IMU_TEMP_KP,
    .ki = IMU_TEMP_KI,
    .kd = IMU_TEMP_KD,
    .i_max = IMU_TEMP_I_MAX,
    .out_max = (float)IMU_TEMP_PWM_FULL_SCALE,
    .i_gap = IMU_TEMP_I_GAP_C,
};

static PID_t temperature_pi;
static uint16_t control_ticks;
static uint8_t overtemperature;

void IMU_Temperature_Init(void)
{
    PID_Init(&temperature_pi, &temperature_pi_config);
    control_ticks = 0U;
    overtemperature = 0U;
    memset((void *)&imu_temperature_stats, 0, sizeof(imu_temperature_stats));

    BSP_PWM_SetPulse(&heater_pwm, 0U);
    BSP_PWM_Start(&heater_pwm);
}

void IMU_Temperature_Tick1ms(void)
{
    float output;
    float temperature;

    if (++control_ticks < IMU_TEMP_CONTROL_TICKS)
        return;

    control_ticks = 0U;
    if (!BMI088_ReadTemperature(&temperature))
    {
        imu_temperature_stats.read_fail_count++;
        return;
    }

    imu_temperature_stats.read_ok_count++;
    imu_temperature_stats.temperature_celsius = temperature;
    if ((temperature >= IMU_TEMP_OVERHEAT_C) || (overtemperature != 0U))
    {
        overtemperature = 1U;
        PID_Init(&temperature_pi, &temperature_pi_config);
        output = 0.0f;
    }
    else
    {
        output = PID_Calculate(&temperature_pi, IMU_TEMP_TARGET_C, temperature);
    }

    if (output < 0.0f)
        output = 0.0f;
    else if (output > (float)IMU_TEMP_PWM_FULL_SCALE)
        output = (float)IMU_TEMP_PWM_FULL_SCALE;

    imu_temperature_stats.overtemperature = overtemperature;
    imu_temperature_stats.pwm_pulse = (uint16_t)(output + 0.5f);
    BSP_PWM_SetPulse(&heater_pwm, imu_temperature_stats.pwm_pulse);
}
