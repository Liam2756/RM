/**
 ******************************************************************************
 * @file    gimbal.h
 * @brief   2027 Hero 小云台控制层接口
 * @note    层级说明：
 *            本文件属于【控制层】，位于设备驱动层之上、应用层之下。
 *            依赖链（单向向下）：
 *              gimbal → pid（算法层）
 *              gimbal → imu_attitude / gyro_calibration / motor_driver（数据与设备层）
 *              gimbal → hero_motor_config（设备驱动层：云台电流下发 + 反馈宿主）
 *            禁止直接操作寄存器 / HAL 外设，不调用应用层。
 *
 *          控制结构（双轴各一套串级 PID；6020 自带电流环，本层不做电流环）：
 *            外环 角度环（rad）  → 目标角速度（rad/s）
 *            内环 速度环（rad/s）→ 电机电流（目标转矩）
 *
 *          姿态量来源（定案：照参考工程 A 分轴，不上 accel 融合 / 四元数）：
 *            Pitch 角度 = GM6020 编码器机械角（绝对、无漂移，相对底盘）
 *            Pitch 角速度 = IMU 陀螺 pitch 轴（不用编码器 RPM——量化粗、抗扰差、超调根源）
 *            Yaw 角度   = GM6020 编码器机械角（无滑环，绕中心有界；不 wrap、不做陀螺积分）
 *            Yaw 角速度 = IMU 陀螺 yaw 轴
 *            accel 运行期不参与姿态，仅上电静态标定里当"静止"判据（见 gyro_calibration）。
 *
 *          PID增益、重力前馈、输出限幅、编码器零点、机械限位、轴映射与全部方向符号
 *          已于2026-08-20完成小云台台架整定，当前参数为成品定案值。
 * @version 4.0
 * @date    2026-8-20
 ******************************************************************************
 */

#ifndef GIMBAL_H
#define GIMBAL_H

#include <pid.h>   /* 算法层：Gimbal_Axis_t 内含 PID_t */

/* ========================================================================== */
/*                              数学常量                                        */
/* ========================================================================== */

#ifndef GIMBAL_PI
#define GIMBAL_PI       3.14159265358979323846f
#define GIMBAL_2PI      (2.0f * GIMBAL_PI)
#endif

/* ========================================================================== */
/*                          控制周期                                            */
/* ========================================================================== */

/** 控制周期（s），须与 1kHz 定时器中断频率一致 */
#define GIMBAL_CONTROL_DT           0.001f  /* 1ms = 1kHz */

/** Pitch速度环独立整定模式：0=启用正常角度环；1=绕过角度环并直给固定目标角速度。 */
#define GIMBAL_PITCH_SPEED_TUNING_MODE  0U
/** Pitch固定目标角速度：仅速度环独立整定模式使用。 */
#define GIMBAL_PITCH_TUNING_OMEGA       10.0f

/* ========================================================================== */
/*                          电机槽位（照 hero_motor_config）                    */
/* ========================================================================== */

/** 云台电机数组索引：0 = Pitch（CAN1，ID2），1 = Yaw（CAN1，ID1），与 g_hero_gimbal_motor 一致 */
#define GIMBAL_PITCH_MOTOR_INDEX    0U
#define GIMBAL_YAW_MOTOR_INDEX      1U

/* ========================================================================== */
/*                          陀螺仪轴映射（小云台成品实标）                      */
/* ========================================================================== */

/** BMI088 gyro[] 到云台轴的映射 + 符号（实测 2026-08-18，随 IMU 安装朝向定）。
 *  实测：gyro[1]=Pitch 低头为正；gyro[2]=Yaw(Z) 左转(CCW)为正。
 *  控制约定（见 Gimbal_SetTargetRate）：Pitch 上仰为正、Yaw 右偏(CW)为正 → 两轴各取反。
 *  符号只在消费点相乘（Gimbal_Control 的 omega 反馈、Gimbal_UpdateSensors 的 Yaw 积分），
 *  s_imu_gyro[] 本身保持原始去零偏值，便于调试 live-watch。 */
#define GIMBAL_GYRO_PITCH_AXIS      1        /**< gyro[1] → Pitch 轴角速度 */
#define GIMBAL_GYRO_YAW_AXIS        2        /**< gyro[2] → Yaw  轴角速度 */
#define GIMBAL_GYRO_PITCH_SIGN      (-1.0f)  /**< 低头为正 → 取反对齐"上仰为正" */
#define GIMBAL_GYRO_YAW_SIGN        (-1.0f)  /**< 左转为正 → 取反对齐"右偏为正" */

/* ========================================================================== */
/*                          Pitch 轴硬件参数（小云台成品实标）                  */
/* ========================================================================== */

/** Pitch 水平位编码值（更换电机后实测 2026-08-20）：炮管水平时的原始编码，用作角度0点 */
#define PITCH_ENCODER_HORIZON       1450U
/** 编码器量程 */
#define PITCH_ENCODER_RANGE         8192U
/** Pitch机械上限（2026-08-20放宽后确认）：炮口最高raw=2570；raw随上仰增大 */
#define PITCH_MAX_ENCODER           2570U
/** Pitch机械下限（2026-08-20放宽后确认）：炮口最低raw=530 */
#define PITCH_MIN_ENCODER           530U
/** Pitch控制上限：比机械上限向内约1°，到此锁住继续上仰的速度指令 */
#define PITCH_CONTROL_MAX_ENCODER   2547U
/** Pitch控制下限：比机械下限向内约1°，到此锁住继续低头的速度指令 */
#define PITCH_CONTROL_MIN_ENCODER   553U
/** 限位释放滞环：反向退出16计数（约0.0123rad/0.70°）后才重新允许原方向速度指令 */
#define PITCH_LIMIT_HYSTERESIS_COUNTS  16U
/** Pitch目标角上限：raw2570相对水平几何角约+0.8590rad，向内保留约1°余量 */
#define PITCH_ANGLE_MAX_RAD         0.841f
/** Pitch目标角下限：raw530相对水平几何角约-0.7056rad，向内保留约1°余量 */
#define PITCH_ANGLE_MIN_RAD         (-0.688f)

/* ========================================================================== */
/*                          Yaw 轴编码器参数（外环角度基准 + 软限位）           */
/* ========================================================================== */

/** Yaw 机械中心 "home"：无滑环，仅一整周可用行程，取编码器量程中点为居中位（2026-08-18）*/
#define GIMBAL_YAW_MIDDLE_ENCODER   4096U
/** 编码器量程（与 Pitch 同型号电机）*/
#define GIMBAL_ENCODER_RANGE        8192U
/** Yaw 编码器符号（实测 2026-08-18）：右转(CW)编码减小 → 取反使"编码器角 +"对齐"右偏为正"，与陀螺内环同向 */
#define GIMBAL_YAW_ENCODER_SIGN     (-1.0f)
/** Yaw 成品软限位幅值（rad，相对中心对称）：无滑环护线，定案为±170°（≈2.97） */
#define GIMBAL_YAW_ANGLE_LIMIT_RAD  2.97f

/* ========================================================================== */
/*                    电机电流输出极性（台架实测，独立于测量符号）              */
/* ========================================================================== */

/** GM6020 电流输出符号：把控制器算出的目标转矩翻到电机【实际正转方向】。
 *  ⚠ 与 GYRO/ENCODER 符号是两码事：那两个把【测量】对齐控制约定（上仰+/右偏+），
 *    这里把【执行】对齐测量——即"正电流是否真的朝控制正方向转"。
 *  内环稳定性 = 本符号 × GYRO 符号：若正电流转出的方向被陀螺读成负 → 内环正反馈 → 疯转。
 *  重力前馈同走此符号，故一个符号同时把内环与 FF 都翻对，无需单独处理。
 *  ⚠ 逐轴独立，两轴电机极性不同（2026-08-19 台架实测）：
 *    - Pitch：+1 稳定，取 -1 时疯转 → 保持 +1。
 *    - Yaw  ：+1 疯转，取 -1 稳定       → 取 -1。 */
#define GIMBAL_PITCH_MOTOR_SIGN     (1.0f)
#define GIMBAL_YAW_MOTOR_SIGN       (-1.0f)

/* ========================================================================== */
/*                          速度环死区                                          */
/* ========================================================================== */

/** 速度环死区阈值：Pitch（rad/s）——覆盖静止陀螺 ±0.035 rad/s 噪声；当前 Ki=0 */
#define GIMBAL_PITCH_OMEGA_DEADBAND 0.05f
/** 速度环死区阈值：Yaw（rad/s）*/
#define GIMBAL_YAW_OMEGA_DEADBAND   0.0f

/* ========================================================================== */
/*                          Pitch 轴 PID 参数（小云台成品定案）                 */
/* ========================================================================== */

#define PITCH_ANGLE_KP              100.0f
#define PITCH_ANGLE_KI              0.0f
#define PITCH_ANGLE_KD              0.0f
#define PITCH_ANGLE_IOUT_MAX        5.0f     /**< 角度环积分限幅（rad/s）*/
#define PITCH_ANGLE_OUT_MAX         10.0f    /**< 角度环输出限幅（rad/s）*/
#define PITCH_ANGLE_IGAP            1000.0f  /**< 积分分离阈值（rad，极大=不分离）*/

#define PITCH_OMEGA_KP              300.0f
#define PITCH_OMEGA_KI              0.0f
#define PITCH_OMEGA_KD              0.0f
#define PITCH_OMEGA_IOUT_MAX        1000.0f  /**< 速度环积分限幅（电流值）*/
#define PITCH_OMEGA_OUT_MAX         6000.0f /**< 速度环输出限幅（电流值）*/
#define PITCH_OMEGA_IGAP            3.0f     /**< 积分分离阈值（rad/s）*/

/** Pitch 重力前馈：水平位(角0)的配平力矩（电流值，此处即 cos 加权的峰值）。
 *  Gimbal_Control 按 cos(pitch_angle_now) 加权下发（§5.6），全行程配平重力。
 *  整定法：把内环增益压到很低，只调本值使炮管在【水平位】大致不垂不飘；
 *  cos 加权会自动把其余角度按比例缩小，正常无需再逐角度试。 */
#define PITCH_EQUILIBRIUM_TORQUE    540.0f

/* ========================================================================== */
/*                          Yaw 轴 PID 参数（小云台成品定案）                   */
/* ========================================================================== */

#define YAW_ANGLE_KP                50.0f
#define YAW_ANGLE_KI                0.0f
#define YAW_ANGLE_KD                0.0f
#define YAW_ANGLE_IOUT_MAX          5.0f
#define YAW_ANGLE_OUT_MAX           10.0f
#define YAW_ANGLE_IGAP              1000.0f  /**< 极大=不分离 */

#define YAW_OMEGA_KP                1300.0f
#define YAW_OMEGA_KI                0.0f
#define YAW_OMEGA_KD                0.0f
#define YAW_OMEGA_IOUT_MAX          1000.0f
#define YAW_OMEGA_OUT_MAX           6000.0f
#define YAW_OMEGA_IGAP              3.0f

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  单轴云台控制状态
 */
typedef struct
{
    float target_angle; /**< 目标角度（rad），由角速度指令累积             */
    PID_t pid_angle;    /**< 外环：角度 PID（角度 → 目标角速度）           */
    PID_t pid_omega;    /**< 内环：角速度 PID（角速度 → 目标转矩）         */
} Gimbal_Axis_t;

typedef enum
{
    GIMBAL_INPUT_MANUAL = 0U,
    GIMBAL_INPUT_VISION = 1U,
} Gimbal_InputMode_t;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  云台模块初始化
 * @note   完成：① 陀螺仪上电静态零偏标定（阻塞约 2s，机器人须水平静止）；
 *               ② Pitch / Yaw 双轴串级 PID 初始化；③ 控制状态复位（目标锁到当前）。
 *         须在 HAL_TIM_Base_Start_IT（使能 1kHz 中断）之前调用，且须在
 *         原 IMU 初始化成功之后（标定依赖可用的 IMU）。
 * @retval 无
 */
void Gimbal_Init(void);

/**
 * @brief  更新云台传感量（每周期无条件调用，含保险档）
 * @note   读 BMI088 + 零偏补偿（gyro[] 供两轴内环角速度反馈；Yaw 角度已改走编码器，
 *         本函数不再做陀螺积分）。无条件采样保证静止（含保险档）时动态零偏 IIR
 *         持续收敛；与力矩下发解耦，本函数不产生任何输出。应在总保险分支【之前】调用。
 * @retval 无
 */
void Gimbal_UpdateSensors(void);

/**
 * @brief  设置云台目标角速度指令（rad/s）
 * @note   由输入链（DT7_Control 等）在 arm 分支调用；本函数只存指令，
 *         正常模式下在Gimbal_Control内积分为目标角；Pitch速度整定模式下
 *         只使用pitch_rate的符号，产生固定目标角速度。
 * @param  yaw_rate    Yaw 轴角速度指令（rad/s），正值为右偏
 * @param  pitch_rate  Pitch 轴角速度指令（rad/s），正值为上仰
 * @retval 无
 */
void Gimbal_SetTargetRate(float yaw_rate, float pitch_rate);

void Gimbal_SetTargetAngle(float yaw_angle, float pitch_angle);

void Gimbal_SetInputMode(Gimbal_InputMode_t mode);

/**
 * @brief  云台控制主函数（arm 分支每周期调用）
 * @note   须在 Gimbal_UpdateSensors 之后调用。执行流程：
 *           1. 正常模式下指令角速度积分为目标角；Pitch速度整定模式绕过该轴积分和角度环
 *           2. 速度整定模式按摇杆方向直给固定目标角速度
 *           3. 固定速度与角度模式共用Pitch编码器滞环锁存，阻止限位附近反复恢复向外速度指令
 *           4. Pitch速度P环叠加余弦重力前馈
 *           5. 经 HeroMotorTx_SetGimbalCurrent 写下发槽位（不直接碰 CAN）
 * @retval 无
 */
void Gimbal_Control(void);

/**
 * @brief  复位云台控制状态（保险分支每周期调用）
 * @note   清 4 组 PID 积分/历史、清指令角速度；Pitch / Yaw 目标角均锁到当前
 *         编码器角（无跳变解保，误差为 0）。本函数不下发电流（保险分支由
 *         HeroMotorTx_ClearAll 统一清零槽位）。
 * @retval 无
 */
void Gimbal_Reset(void);

/**
 * @brief  获取 Yaw 轴相对底盘的偏转角（rad），范围 (-π, π]
 * @note   基于 Yaw 6020 编码器计算，0 表示云台正对底盘前方，正值为 CCW。
 *         供后续底盘随动使用（当前阶段底盘未接入）。
 * @retval Yaw 相对角（rad）
 */
float Gimbal_GetYawRelativeAngle(void);

/* ========================================================================== */
/*                          调试：Pitch 串级环 SerialPlot 采样                  */
/* ========================================================================== */

/** Pitch 调试采样通道数（目标/实际 角度、角速度、电流 各一对）*/
#define GIMBAL_PITCH_PLOT_CHANNELS  6U

/**
 * @brief  取 Pitch 串级环最近一次采样（供上层 SerialPlot 下发，调试用）
 * @param  out  长度 >= GIMBAL_PITCH_PLOT_CHANNELS 的 float 数组，按下列顺序写入：
 *               [0] 目标角(rad)   [1] 实际角(rad)
 *               [2] 目标角速度(rad/s) [3] 实际角速度(rad/s)
 *               [4] 目标电流(控制系约定,上仰+) [5] 实际电流(反馈,已折算同约定)
 * @note   仅在 arm(Gimbal_Control 被调用)周期刷新；保险/离线周期返回上一次值。
 *         全部通道统一为控制正方向约定(上仰+)，故 6 路同号可直接叠图对读。
 */
void Gimbal_DebugGetPitchPlot(float *out);

#endif /* GIMBAL_H */
