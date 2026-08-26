/**
 ******************************************************************************
 * @file    app.c
 * @brief   英雄机器人应用层初始化与 1 kHz 调度
 * @note    调度采用集中式流水线（见构建方案 §5.4）：
 *            输入源只把遥控指令翻译为各子系统目标，统一任务再计算各环，
 *            最后经发送分组模块一次性下发。
 *
 *          保险归属：DT7 左拨杆 s[0] 三档（UP/MID/DOWN=保险）的分发【收在
 *          DT7_Control 内】（照参考工程 A 集中到一个 switch），返回 SAFE/ARMED；
 *          调度器据此决定是否算环。本文件只保留【离线拦截】——DT7 掉线时翻译层
 *          不可信，直接控制层复位强制 SAFE。安全档：不算环、槽位保持清零后照常下发。
 *
 *          当前：摩擦轮 + 拨弹盘（发射联锁）+ 云台三机构参与控制；云台力矩下发已接入
 *          arm 分支（低权限首上电档，gimbal.h 增益已压小；方向符号待台架校正，见 §5.5）；
 *          底盘后续阶段接入。
 * @version 3.2
 * @date    2026-8-19
 ******************************************************************************
 */

#include <app.h>
#include <bsp_debug.h>
#include <bsp_uart.h>
#include <DT7.h>
#include <dt7_driver.h>
#include <filter.h>
#include <gimbal.h>
#include <hero_motor_config.h>
#include <tim.h>
#include <usart.h>   /* 调试 SerialPlot 下发口句柄（huart1，见下方 PLOT 宏）*/
#include <math.h>
#include <vision_uart.h>

/* ========================================================================== */
/*        调试：Pitch 串级环 SerialPlot 下发（整定用，装车前置 0 关闭）         */
/* ========================================================================== */
/* 调试输出使用 USART1（PA9 TX，961200）。6 路数据以 1kHz 下发时 DMA 不会撞下一拍。 */
#define GIMBAL_DEBUG_PLOT_ENABLE   1
#define GIMBAL_DEBUG_PLOT_UART     (&huart1)
#define GIMBAL_DEBUG_PLOT_DIVIDER  1U   /* 每 N 个 1kHz 周期发一帧（USART1 取 1 = 全速）*/

void App_Init(void)
{
    DT7_DriverInit();

    if (HeroMotorConfig_Init() != HAL_OK)
    {
        Error_Handler();
    }
    if (can_filter_init() != HAL_OK)
    {
        Error_Handler();
    }

    HeroMotorTx_ClearAll();
    if (HeroMotorTx_FlushAll() != HAL_OK)
    {
        Error_Handler();
    }

    /* 云台初始化：复用原 IMU 的传感器读取，并进行陀螺零偏静态标定。 */
    Vision_UART_Init();
    Gimbal_Init();
}

void App_ControlTask1ms(void)
{
    DT7_ctrl_t ctrl;
    DT7_Mode_t mode;
    uint8_t dt7_offline = DT7_CheckOffline();
    uint8_t vision_mode;
    Vision_Target_t vision_target;

    DT7_GetSnapshot(&ctrl);
    vision_mode = (uint8_t)((dt7_offline == 0U) &&
                            (ctrl.rc.s[1] == DT7_SWITCH_UP));
    Gimbal_SetInputMode(vision_mode != 0U ? GIMBAL_INPUT_VISION : GIMBAL_INPUT_MANUAL);

    if (Vision_UART_GetNewTarget(&vision_target) != 0U)
    {
        if (vision_mode != 0U)
        {
            float horizontal_mm = sqrtf(vision_target.x_mm * vision_target.x_mm +
                                         vision_target.y_mm * vision_target.y_mm);
            float yaw_target = -atan2f(vision_target.y_mm, vision_target.x_mm);
            float pitch_target = atan2f(vision_target.z_mm, horizontal_mm);
            Gimbal_SetTargetAngle(yaw_target, pitch_target);
        }
    }

    /* 蓝灯：左拨杆处于保险档（s[0]=DOWN）指示；离线时快照已归中立，蓝灯自然熄。 */
    BSP_Debug_LEDSet(BSP_DEBUG_LED_B, (ctrl.rc.s[0] == DT7_SWITCH_DOWN) ? 1U : 0U);

    /* 云台传感更新：读 IMU + 零偏补偿（Yaw 角度已改走编码器，此处不再积分）。
       无条件采样（含保险/离线）保证静止时动态零偏持续收敛；本调用不产生力矩，安全无副作用。 */
    Gimbal_UpdateSensors();

    /* 本周期先清零全部发送槽位；解保时各环再覆盖各自槽位，保险/离线则保持清零 = 零输出。 */
    HeroMotorTx_ClearAll();

    if (dt7_offline != 0U)
    {
        /* 离线拦截（本文件唯一的安全判定）：翻译层不可信，直接控制层复位强制 SAFE。
           清目标/PID 保证重连无跳变；槽位已清零→本周期零输出。 */
        Gimbal_Reset();
        mode = DT7_MODE_SAFE;
    }
    else
    {
        /* 在线：左拨杆 s[0] 三档（UP/MID/DOWN=保险）的分发全部由 DT7_Control 决定，
           它写好各子系统目标（或保险档复位）并返回本周期模式。 */
        mode = DT7_Control(&ctrl);
    }

    /* 仅解保才计算各环（覆盖槽位）；保险/离线时不算环，槽位保持清零 → 滑行到零电流。 */
    if (mode == DT7_MODE_ARMED)
    {
        /* 云台力矩下发（已接，低权限首上电）：积分目标角 + 双环 PID + 电流下发，覆盖云台槽位。
           gimbal.h 增益/输出限幅/重力前馈已压到低权限起整定档；方向符号（DT7_GIMBAL_*_SIGN
           与 gimbal.h GYRO/ENCODER 符号）待台架首上电校正。保险/离线时不进本分支 → 槽位保持
           清零 → 0 电流 limp（Gimbal_Reset 已锁 target，重连无跳变解保）。见 §5.4/§5.5。 */
        Gimbal_Control();
    }

    (void)HeroMotorTx_FlushAll();

#if GIMBAL_DEBUG_PLOT_ENABLE
    /* Pitch 串级环采样下发（降采样后经 DMA 送 SerialPlot）。数据仅在 arm 周期刷新，
       保险/离线时为上一次值（各环停算），对整定无碍——整定始终在 arm 下进行。 */
    {
        static uint16_t s_plot_div;
        if (++s_plot_div >= GIMBAL_DEBUG_PLOT_DIVIDER)
        {
            float plot[GIMBAL_PITCH_PLOT_CHANNELS];
            s_plot_div = 0U;
            Gimbal_DebugGetPitchPlot(plot);
            BSP_Debug_SerialPlot(GIMBAL_DEBUG_PLOT_UART, plot, GIMBAL_PITCH_PLOT_CHANNELS);
        }
    }
#endif
}
