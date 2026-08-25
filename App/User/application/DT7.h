/**
 ******************************************************************************
 * @file    DT7.h
 * @brief   DT7 输入到英雄控制目标的应用层接口
 * @note    小云台双轴通道、方向、灵敏度与死区已于2026-08-20完成台架定案。
 * @version 3.0
 * @date    2026-8-20
 ******************************************************************************
 */

#ifndef DT7_H
#define DT7_H

#include <dt7_driver.h>

/* ========================================================================== */
/*        摇杆 → 云台角速度 翻译策略（应用层，照参考工程 A 的映射）             */
/* ========================================================================== */

/** 右摇杆水平 → Yaw 角速度通道（照 A）*/
#define DT7_CH_YAW              0U
/** 右摇杆垂直 → Pitch 角速度通道（照 A）*/
#define DT7_CH_PITCH            1U

/** 通道相对中值的偏移（±660），中值宏取自设备驱动层 dt7_driver.h */
#define DT7_CH_OFFSET(ctrl, idx) \
    ((int16_t)((ctrl)->rc.ch[(idx)] - DT7_CHANNEL_CENTER))

/** 摇杆灵敏度：通道偏移(±660) → 角速度指令(rad/s) 的比例，照 A（0.01）*/
#define DT7_GIMBAL_SENSITIVITY  0.010f
/** 云台摇杆中位死区（通道计数）：偏移绝对值 <= 5 时按 0 处理 */
#define DT7_GIMBAL_STICK_DEADBAND 5

/* Yaw/Pitch摇杆方向符号（小云台成品台架实标）：推杆右→云台右，推杆上→炮口上。 */
#define DT7_GIMBAL_YAW_SIGN     (1.0f)
#define DT7_GIMBAL_PITCH_SIGN   (1.0f)

/* 单轴调试通道开关：成品状态两轴均为0（开启）；置1可临时关闭对应轴摇杆。 */
#define DT7_GIMBAL_YAW_STICK_DISABLED    0U
#define DT7_GIMBAL_PITCH_STICK_DISABLED  0U

/* ========================================================================== */
/*                          左拨杆分发结果                                       */
/* ========================================================================== */

/**
 * @brief  DT7_Control 对本周期的分发结论
 * @note   调度器据此决定是否计算各控制环：SAFE 则跳过（槽位保持清零→零输出），
 *         ARMED 则运行各环覆盖槽位。安全与否的【判定】收在 DT7_Control 内。
 */
typedef enum
{
    DT7_MODE_SAFE  = 0, /**< 保险：控制层已复位，本周期不计算环、不产生力矩 */
    DT7_MODE_ARMED = 1  /**< 解保：目标已写好，调度器应计算各环并下发 */
} DT7_Mode_t;

/**
 * @brief  DT7 输入源翻译 + 左拨杆模式分发（应用层唯一的拨杆语义所有者）
 * @param  dt7_ctrl  本周期 DT7 一致快照（仅在线时调用；离线由 app.c 拦截）
 * @retval DT7_MODE_SAFE / DT7_MODE_ARMED，见枚举说明
 */
DT7_Mode_t DT7_Control(const DT7_ctrl_t *dt7_ctrl);

#endif /* DT7_H */
