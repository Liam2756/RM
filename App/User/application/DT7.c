/**
 ******************************************************************************
 * @file    DT7.c
 * @brief   DT7 输入到英雄控制目标的应用层翻译
 * @note    层级说明：
 *            本文件属于【应用层】，把 DT7 一致快照翻译为各子系统目标
 *            （只写目标，不计算环路、不发送 CAN）。向下依赖设备驱动层
 *            （DT7 快照类型）与控制层（摩擦轮/拨弹指令接口）。
 *
 *          §5.4 主控制入口的英雄雏形。本文件是左拨杆 s[0] 语义的【唯一所有者】：
 *          以 switch(s[0]) 分发 UP=全功能 / MID=运动云台 / DOWN=保险 三档（照参考
 *          工程 A 的组织），并返回本周期模式（SAFE/ARMED）供调度器决定是否算环。
 *          保险档只走控制层复位（不直捅 CAN，区别于 A），由发送分组在输出级清零。
 * @version 4.0
 * @date    2026-8-19
 * @note    v3.0：翻译逻辑由 app.c 的临时宿主 App_DT7Control 迁回本文件
 *          （其过渡使命结束，归位到应用层该在的地方）。
 * @note    v4.0：左拨杆保险/arm 分发由 app.c 上游总闸收回本文件（照 A 集中到
 *          DT7_Control 的 switch(s[0])）；app.c 仅保留离线拦截。新增右摇杆→云台
 *          角速度翻译（照 A 映射，方向符号见 DT7.h 台架校向）。
 ******************************************************************************
 */

#include <DT7.h>
#include <gimbal.h>

/**
 * @brief  四轮摩擦轮统一设目标转速（内部辅助）
 */
/**
 * @brief  对云台摇杆通道偏移施加中位死区（内部辅助）
 */
static int16_t DT7_GimbalApplyDeadband(int16_t offset)
{
    return ((offset >= -DT7_GIMBAL_STICK_DEADBAND) &&
            (offset <=  DT7_GIMBAL_STICK_DEADBAND)) ? 0 : offset;
}

/**
 * @brief  右摇杆 → 云台角速度指令翻译（内部辅助）
 * @note   照 A 映射：水平(ch0)→Yaw、垂直(ch1)→Pitch，偏移×灵敏度×方向符号。
 *         正常模式把偏移按灵敏度翻译为目标角速度；Pitch速度整定模式只使用垂直摇杆
 *         指令的正负方向，直给固定目标角速度。方向符号见DT7.h。
 *         台架校向说明——英雄陀螺/编码相对 A 已翻转，默认符号大概率需在台架翻。
 */
static void DT7_TranslateGimbal(const DT7_ctrl_t *dt7_ctrl)
{
#if DT7_GIMBAL_YAW_STICK_DISABLED
    int16_t yaw_offset = 0; /* 台架：Yaw 只锁位，单独整定 Pitch */
#else
    int16_t yaw_offset = DT7_GimbalApplyDeadband(
        DT7_CH_OFFSET(dt7_ctrl, DT7_CH_YAW));
#endif
#if DT7_GIMBAL_PITCH_STICK_DISABLED
    int16_t pitch_offset = 0;
#else
    int16_t pitch_offset = DT7_GimbalApplyDeadband(
        DT7_CH_OFFSET(dt7_ctrl, DT7_CH_PITCH));
#endif
    Gimbal_SetTargetRate(
        DT7_GIMBAL_YAW_SIGN   * (float)yaw_offset   * DT7_GIMBAL_SENSITIVITY,
        DT7_GIMBAL_PITCH_SIGN * (float)pitch_offset * DT7_GIMBAL_SENSITIVITY);
}

/**
 * @brief  DT7 输入源翻译 + 左拨杆模式分发
 * @note   照参考工程 A 的组织，以 switch(s[0]) 集中分发三档；但保险档【不直捅 CAN】
 *         （区别于 A），只调控制层复位，实际清零交给发送分组（输出级）：
 *           - UP  （全功能）：摩擦轮跑标定转速 + 发射指令(s[1]) + 云台角速度 → ARMED
 *           - MID （运动）  ：摩擦轮停 + 不发射（无摩擦轮不送弹）+ 云台角速度 → ARMED
 *           - DOWN（保险）  ：三机构控制层复位（清目标/PID、云台 target 归位）→ SAFE
 *           - 其它（含离线复位后的中立/非法码）：按保险处理 → SAFE
 *         返回模式供调度器决定是否计算各环（SAFE 时槽位保持清零 = 零输出/滑行）。
 */
DT7_Mode_t DT7_Control(const DT7_ctrl_t *dt7_ctrl)
{
    switch (dt7_ctrl->rc.s[0])
    {
    case DT7_SWITCH_UP:
    case DT7_SWITCH_MID:
        DT7_TranslateGimbal(dt7_ctrl);
        return DT7_MODE_ARMED;

    case DT7_SWITCH_DOWN:
    default:
        Gimbal_Reset();
        return DT7_MODE_SAFE;
    }
}
