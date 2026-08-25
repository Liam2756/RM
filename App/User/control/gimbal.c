/**
 ******************************************************************************
 * @file    gimbal.c
 * @brief   英雄云台控制层实现（双轴串级 PID）
 * @note    职责边界：
 *            【可以调用】原 IMU 数据接口、设备驱动层（gyro_calibration / motor_driver /
 *                        hero_motor_config）、算法层（PID / 角度数学）
 *            【禁止】直接操作寄存器 / HAL 外设、不调用应用层
 *
 *          姿态量与整定说明见 gimbal.h 顶部。相较参考工程 A 的 gimbal.c，本文件
 *          修正了其"声明 gyro[] 却从未调原 IMU 数据接口/GyroBias_Correct、拿未初始化
 *          值进环"的半成品缺陷：IMU 读取 + 零偏补偿统一在 Gimbal_UpdateSensors
 *          里完成（每周期无条件调用），控制环消费去零偏真值。
 * @version 3.0
 * @date    2026-8-18
 ******************************************************************************
 */

#include <gimbal.h>
#include <math.h>              /* fabsf, fmodf, cosf */
#include <imu_attitude.h>      /* 复用原 IMU 的单次读取缓存 */
#include <gyro_calibration.h>  /* 设备驱动层：GyroBias_StaticCalib / Correct */
#include <hero_motor_config.h> /* 设备驱动层：g_hero_gimbal_motor / 云台电流下发 */

/* ========================================================================== */
/*                          PID 配置（起整定参考，见 gimbal.h）                 */
/* ========================================================================== */

static const PID_Config_t s_pitch_angle_cfg = {
    .kp = PITCH_ANGLE_KP, .ki = PITCH_ANGLE_KI, .kd = PITCH_ANGLE_KD,
    .i_max = PITCH_ANGLE_IOUT_MAX, .out_max = PITCH_ANGLE_OUT_MAX,
    .i_gap = PITCH_ANGLE_IGAP,
};
static const PID_Config_t s_pitch_omega_cfg = {
    .kp = PITCH_OMEGA_KP, .ki = PITCH_OMEGA_KI, .kd = PITCH_OMEGA_KD,
    .i_max = PITCH_OMEGA_IOUT_MAX, .out_max = PITCH_OMEGA_OUT_MAX,
    .i_gap = PITCH_OMEGA_IGAP,
};
static const PID_Config_t s_yaw_angle_cfg = {
    .kp = YAW_ANGLE_KP, .ki = YAW_ANGLE_KI, .kd = YAW_ANGLE_KD,
    .i_max = YAW_ANGLE_IOUT_MAX, .out_max = YAW_ANGLE_OUT_MAX,
    .i_gap = YAW_ANGLE_IGAP,
};
static const PID_Config_t s_yaw_omega_cfg = {
    .kp = YAW_OMEGA_KP, .ki = YAW_OMEGA_KI, .kd = YAW_OMEGA_KD,
    .i_max = YAW_OMEGA_IOUT_MAX, .out_max = YAW_OMEGA_OUT_MAX,
    .i_gap = YAW_OMEGA_IGAP,
};

/* ========================================================================== */
/*                          模块私有状态                                        */
/* ========================================================================== */

static Gimbal_Axis_t s_yaw_axis;           /**< Yaw  轴控制状态（含双环 PID）*/
static Gimbal_Axis_t s_pitch_axis;         /**< Pitch 轴控制状态（含双环 PID）*/
static GyroBias_t    s_gyro_bias;          /**< 陀螺零偏（StaticCalib 后持续动态更新）*/

static float         s_imu_gyro[3];        /**< 去零偏后的陀螺三轴（rad/s）*/
static float         s_imu_accel[3];       /**< 加速度三轴（m/s²，仅供零偏静止判据）*/

static float         s_cmd_yaw_rate;       /**< 输入链给的 Yaw 角速度指令（rad/s）*/
static float         s_cmd_pitch_rate;     /**< 输入链给的 Pitch 角速度指令（rad/s）*/
static uint8_t       s_pitch_upper_blocked; /**< 已进入上限空挡区，锁住继续上仰指令 */
static uint8_t       s_pitch_lower_blocked; /**< 已进入下限空挡区，锁住继续低头指令 */

static float         s_pitch_plot[GIMBAL_PITCH_PLOT_CHANNELS]; /**< Pitch 串级环调试采样（见 Gimbal_DebugGetPitchPlot）*/

/* ========================================================================== */
/*                              私有函数声明                                    */
/* ========================================================================== */

#if !GIMBAL_PITCH_SPEED_TUNING_MODE
static float Gimbal_ShortestError(float target, float measure);
#endif
static float Gimbal_GetPitchAngle(void);
static float Gimbal_GetYawAngle(void);
static void  Gimbal_ResetControlState(void);

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  云台模块初始化
 */
void Gimbal_Init(void)
{
    /* 陀螺仪上电静态标定（阻塞 ~2s，须在定时器启动前、机器人水平静止时调用）*/
    GyroBias_StaticCalib(&s_gyro_bias);

    /* 双轴串级 PID 初始化 + 控制状态复位（两轴目标均锁到当前编码器角）*/
    Gimbal_ResetControlState();
}

/**
 * @brief  更新云台传感量（每周期无条件调用）
 */
void Gimbal_UpdateSensors(void)
{
    /* 读 IMU 并原地做零偏补偿：gyro[] 出来即去零偏真值（供两轴内环角速度反馈）*/
    (void)IMU_Attitude_GetLatestSensor(s_imu_gyro, s_imu_accel);
    GyroBias_Correct(&s_gyro_bias, s_imu_gyro, s_imu_accel);
}

/**
 * @brief  设置云台目标角速度指令
 */
void Gimbal_SetTargetRate(float yaw_rate, float pitch_rate)
{
    s_cmd_yaw_rate   = yaw_rate;
    s_cmd_pitch_rate = pitch_rate;
}

/**
 * @brief  云台控制主函数（arm 分支每周期调用，须在 Gimbal_UpdateSensors 之后）
 */
void Gimbal_Control(void)
{
    /* 本轴角速度反馈：来自 IMU 陀螺（非编码器 RPM），乘轴符号对齐控制正方向 */
    float pitch_omega = GIMBAL_GYRO_PITCH_SIGN * s_imu_gyro[GIMBAL_GYRO_PITCH_AXIS];
    float yaw_omega   = GIMBAL_GYRO_YAW_SIGN   * s_imu_gyro[GIMBAL_GYRO_YAW_AXIS];

    /* ── Step 1：正常模式把指令角速度积分为目标角；速度整定模式绕过Pitch积分 ── */
#if !GIMBAL_PITCH_SPEED_TUNING_MODE
    s_pitch_axis.target_angle += s_cmd_pitch_rate * GIMBAL_CONTROL_DT;
#endif
    s_yaw_axis.target_angle   += s_cmd_yaw_rate   * GIMBAL_CONTROL_DT;

#if !GIMBAL_PITCH_SPEED_TUNING_MODE
    /* Pitch 目标角机械限幅（角度环之前截断）*/
    if (s_pitch_axis.target_angle > PITCH_ANGLE_MAX_RAD)
    {
        s_pitch_axis.target_angle = PITCH_ANGLE_MAX_RAD;
    }
    if (s_pitch_axis.target_angle < PITCH_ANGLE_MIN_RAD)
    {
        s_pitch_axis.target_angle = PITCH_ANGLE_MIN_RAD;
    }
#endif

    /* Yaw 目标角软限位（无滑环护线，相对中心 ±LIMIT 对称截断；本轴有界、不 wrap）*/
    if (s_yaw_axis.target_angle > GIMBAL_YAW_ANGLE_LIMIT_RAD)
    {
        s_yaw_axis.target_angle = GIMBAL_YAW_ANGLE_LIMIT_RAD;
    }
    if (s_yaw_axis.target_angle < -GIMBAL_YAW_ANGLE_LIMIT_RAD)
    {
        s_yaw_axis.target_angle = -GIMBAL_YAW_ANGLE_LIMIT_RAD;
    }

    /* ── Step 2：速度整定模式直给固定目标速度；正常模式执行Pitch角度外环 ── */
    float pitch_angle_now = Gimbal_GetPitchAngle();
#if GIMBAL_PITCH_SPEED_TUNING_MODE
    float target_omega_pitch;
    s_pitch_axis.target_angle = pitch_angle_now; /* 调试图中目标角跟随实测，明确角度环未参与 */
    if (s_cmd_pitch_rate > 0.0f)
    {
        target_omega_pitch = GIMBAL_PITCH_TUNING_OMEGA;
    }
    else if (s_cmd_pitch_rate < 0.0f)
    {
        target_omega_pitch = -GIMBAL_PITCH_TUNING_OMEGA;
    }
    else
    {
        target_omega_pitch = 0.0f;
    }
#else
    /* Pitch：机械行程约89.65°，经"虚拟测量"喂线性PID（有界区间内等价于线性误差）*/
    float pitch_virtual_measure =
        pitch_angle_now + Gimbal_ShortestError(s_pitch_axis.target_angle, pitch_angle_now);
    float target_omega_pitch =
        PID_Calculate(&s_pitch_axis.pid_angle, pitch_virtual_measure, pitch_angle_now);
#endif

    /* Yaw：编码器角以中心 4096 为 0、有界；无滑环故走线性误差（穿中心回中，绝不跨硬止点）*/
    float yaw_angle_now    = Gimbal_GetYawAngle();
    float target_omega_yaw =
        PID_Calculate(&s_yaw_axis.pid_angle, s_yaw_axis.target_angle, yaw_angle_now);
    /* Pitch控制限位比机械硬止点向内约1°。固定速度和角度模式共用锁存滞环，避免
       编码器在阈值附近跳动时反复恢复向外目标速度；反向退出约0.70°后才释放。 */
    uint16_t pitch_encoder = g_hero_gimbal_motor[GIMBAL_PITCH_MOTOR_INDEX].rotor_mechanical_angle;
    if (pitch_encoder >= PITCH_CONTROL_MAX_ENCODER)
    {
        s_pitch_upper_blocked = 1U;
    }
    else if ((pitch_encoder <= (PITCH_CONTROL_MAX_ENCODER - PITCH_LIMIT_HYSTERESIS_COUNTS)) &&
             (target_omega_pitch <= 0.0f))
    {
        s_pitch_upper_blocked = 0U;
    }
    if (pitch_encoder <= PITCH_CONTROL_MIN_ENCODER)
    {
        s_pitch_lower_blocked = 1U;
    }
    else if ((pitch_encoder >= (PITCH_CONTROL_MIN_ENCODER + PITCH_LIMIT_HYSTERESIS_COUNTS)) &&
             (target_omega_pitch >= 0.0f))
    {
        s_pitch_lower_blocked = 0U;
    }
    if (((s_pitch_upper_blocked != 0U) && (target_omega_pitch > 0.0f)) ||
        ((s_pitch_lower_blocked != 0U) && (target_omega_pitch < 0.0f)))
    {
        target_omega_pitch = 0.0f;
    }
    /* 调试用：留存死区改写前、机械限位处理后的 Pitch 目标角速度。 */
    float pitch_target_omega_cmd = target_omega_pitch;

    /* ── Step 3：速度环（内环）+ 死区 ── 误差极小时用实测代替目标，抑制静止陀螺噪声 */
    if (fabsf(target_omega_pitch - pitch_omega) <= GIMBAL_PITCH_OMEGA_DEADBAND)
    {
        target_omega_pitch = pitch_omega;
    }
    if (fabsf(target_omega_yaw - yaw_omega) <= GIMBAL_YAW_OMEGA_DEADBAND)
    {
        target_omega_yaw = yaw_omega;
    }

    /* Pitch速度反馈叠加重力前馈：水平位最大，随俯仰角按cos变化。 */
    float pitch_feedback =
        PID_Calculate(&s_pitch_axis.pid_omega, target_omega_pitch, pitch_omega);
    float pitch_gravity_ff = PITCH_EQUILIBRIUM_TORQUE * cosf(pitch_angle_now);
    float torque_pitch = pitch_feedback + pitch_gravity_ff;
    if (torque_pitch > PITCH_OMEGA_OUT_MAX)
    {
        torque_pitch = PITCH_OMEGA_OUT_MAX;
    }
    else if (torque_pitch < -PITCH_OMEGA_OUT_MAX)
    {
        torque_pitch = -PITCH_OMEGA_OUT_MAX;
    }
    float torque_yaw =
        PID_Calculate(&s_yaw_axis.pid_omega, target_omega_yaw, yaw_omega);

    /* ── Step 4：写下发槽位（经电机 TX 层，不直接碰 CAN）──
       末乘 *_MOTOR_SIGN：把目标转矩翻到电机实际正转方向（执行极性对齐测量）。
       与 GYRO/ENCODER 符号相互独立；首上电台架校向，见 gimbal.h 说明。 */
    (void)HeroMotorTx_SetGimbalCurrent(GIMBAL_PITCH_MOTOR_INDEX,
                                       (int16_t)(GIMBAL_PITCH_MOTOR_SIGN * torque_pitch));
    (void)HeroMotorTx_SetGimbalCurrent(GIMBAL_YAW_MOTOR_INDEX,
                                       (int16_t)(GIMBAL_YAW_MOTOR_SIGN * torque_yaw));

    /* ── 调试采样：Pitch 串级环 6 路，统一折算到控制正方向约定（上仰+）── */
    s_pitch_plot[0] = s_pitch_axis.target_angle; /* 目标角(rad) */
    s_pitch_plot[1] = pitch_angle_now;           /* 实际角(rad) */
    s_pitch_plot[2] = pitch_target_omega_cmd;    /* 目标角速度(rad/s，死区改写前) */
    s_pitch_plot[3] = pitch_omega;               /* 实际角速度(rad/s) */
    s_pitch_plot[4] = torque_pitch;              /* 目标电流(控制系，含重力前馈) */
    s_pitch_plot[5] = GIMBAL_PITCH_MOTOR_SIGN *
                      (float)g_hero_gimbal_motor[GIMBAL_PITCH_MOTOR_INDEX].torque_current;
}

/**
 * @brief  复位云台控制状态（保险分支每周期调用）
 */
void Gimbal_Reset(void)
{
    Gimbal_ResetControlState();
}

/**
 * @brief  获取 Yaw 轴相对底盘的偏转角
 */
float Gimbal_GetYawRelativeAngle(void)
{
    uint16_t raw_encoder = g_hero_gimbal_motor[GIMBAL_YAW_MOTOR_INDEX].rotor_mechanical_angle;

    /* 减去中立值后取模，中立位置归零 */
    uint16_t comp_encoder = (uint16_t)((raw_encoder
                                        + GIMBAL_ENCODER_RANGE / 2U
                                        - GIMBAL_YAW_MIDDLE_ENCODER)
                                       % GIMBAL_ENCODER_RANGE);

    /* [0,8191] → (-π, π]：中立=0，CCW 为正 */
    return ((float)comp_encoder / (float)GIMBAL_ENCODER_RANGE) * GIMBAL_2PI - GIMBAL_PI;
}

/**
 * @brief  取 Pitch 串级环最近一次采样（调试，见头文件说明）
 */
void Gimbal_DebugGetPitchPlot(float *out)
{
    uint8_t i;

    if (out == 0)
    {
        return;
    }
    for (i = 0U; i < GIMBAL_PITCH_PLOT_CHANNELS; i++)
    {
        out[i] = s_pitch_plot[i];
    }
}

/* ========================================================================== */
/*                              私有函数实现                                    */
/* ========================================================================== */

/**
 * @brief  从 measure 到 target 的最短路径误差，结果在 (-π, π]
 * @note   先平移到 [0,2π) 再减 π，避免跨 0/2π 边界跳变。
 */
#if !GIMBAL_PITCH_SPEED_TUNING_MODE
static float Gimbal_ShortestError(float target, float measure)
{
    float delta = fmodf(target - measure + GIMBAL_PI, GIMBAL_2PI);
    if (delta < 0.0f)
    {
        delta += GIMBAL_2PI;
    }
    return delta - GIMBAL_PI;
}
#endif

/**
 * @brief  由 Pitch 6020 编码器计算当前 Pitch 角（rad），范围 (-π, π]，上仰为正
 * @note   以水平位编码PITCH_ENCODER_HORIZON(1450)为0点，raw随上仰增大（实测2570=最高）。
 *         机械行程约89.65°，正常不触发wrap；wrap仅防越界读数。
 */
static float Gimbal_GetPitchAngle(void)
{
    uint16_t raw_encoder = g_hero_gimbal_motor[GIMBAL_PITCH_MOTOR_INDEX].rotor_mechanical_angle;
    float    angle       = ((float)raw_encoder - (float)PITCH_ENCODER_HORIZON)
                           / (float)PITCH_ENCODER_RANGE * GIMBAL_2PI;
    /* wrap 到 (-π, π] */
    angle = fmodf(angle + GIMBAL_PI, GIMBAL_2PI);
    if (angle < 0.0f)
    {
        angle += GIMBAL_2PI;
    }
    return angle - GIMBAL_PI;
}

/**
 * @brief  由 Yaw 6020 编码器计算当前 Yaw 角（rad），范围 (-π, π]，右偏(CW)为正
 * @note   以机械中心 GIMBAL_YAW_MIDDLE_ENCODER(=4096) 为 0 点。实测右转编码减小，
 *         故乘 GIMBAL_YAW_ENCODER_SIGN 使"右偏为正"，与陀螺内环同向。
 *         无滑环：本轴有界（软限位 ±GIMBAL_YAW_ANGLE_LIMIT_RAD），不做环形 wrap；
 *         线性误差天然穿中心回中，绝不跨 ±180°（编码 0/8192 处）的线缆硬止点。
 */
static float Gimbal_GetYawAngle(void)
{
    uint16_t raw_encoder = g_hero_gimbal_motor[GIMBAL_YAW_MOTOR_INDEX].rotor_mechanical_angle;
    float    angle       = ((float)raw_encoder - (float)GIMBAL_YAW_MIDDLE_ENCODER)
                           / (float)GIMBAL_ENCODER_RANGE * GIMBAL_2PI;
    /* wrap 到 (-π, π] 后乘符号（右偏为正）*/
    angle = fmodf(angle + GIMBAL_PI, GIMBAL_2PI);
    if (angle < 0.0f)
    {
        angle += GIMBAL_2PI;
    }
    return GIMBAL_YAW_ENCODER_SIGN * (angle - GIMBAL_PI);
}

/**
 * @brief  复位控制状态：清 PID、清指令、目标锁到当前测量（无跳变解保）
 */
static void Gimbal_ResetControlState(void)
{
    PID_Init(&s_pitch_axis.pid_angle, &s_pitch_angle_cfg);
    PID_Init(&s_pitch_axis.pid_omega, &s_pitch_omega_cfg);
    PID_Init(&s_yaw_axis.pid_angle, &s_yaw_angle_cfg);
    PID_Init(&s_yaw_axis.pid_omega, &s_yaw_omega_cfg);

    s_cmd_yaw_rate   = 0.0f;
    s_cmd_pitch_rate = 0.0f;
    s_pitch_upper_blocked = 0U;
    s_pitch_lower_blocked = 0U;

    /* 两轴目标均锁到当前编码器角，解保时角度误差为 0，不产生回中动作。 */
    s_pitch_axis.target_angle = Gimbal_GetPitchAngle();
    s_yaw_axis.target_angle   = Gimbal_GetYawAngle();
}
