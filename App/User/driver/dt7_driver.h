/**
 ******************************************************************************
 * @file    dt7_driver.h
 * @brief   DT7/DR16 遥控器 SBUS 数据解析接口（设备驱动层）
 * @note    层级说明：
 *            本文件属于【设备驱动层】，向下依赖 BSP UART（空闲中断 + DMA 收帧），
 *            向上仅暴露快照/离线查询，不反向依赖控制层或应用层。
 *
 *          SBUS 帧：18 字节、约 14ms 一帧、无 CRC。buf[0..5] 打包 4 个 11 位
 *          摇杆通道与两个 2 位拨杆，buf[16..17] 为拨轮通道 ch[4]。
 *          buf[6..15]（鼠标/键盘）不参与本工程控制，此处不解析。
 *
 *          通道值域 [364, 1684]、中值 1024，物理约对应 [-660, +660]。
 *          拨杆 2 位码值：上 = 1、中 = 3、下 = 2（0 为健康遥控器不发送的未定义值）。
 * @version 2.2
 * @date    2026-8-16
 * @note    v2.1：删除无人使用的鼠标/键盘字段，补全 doxygen。
 * @note    v2.2：在线/离线判定改用算法层 Freshness 看门狗（超时与上线去抖
 *          由 DT7_FRESH_* 参数配置）；DT7_CheckOffline 不再收超时入参。
 ******************************************************************************
 */

#ifndef DT7_DRIVER_H
#define DT7_DRIVER_H

#include <bsp_uart.h>
#include <stdint.h>

#define DT7_FRAME_LENGTH       18U
#define DT7_CHANNEL_COUNT      5U
#define DT7_SWITCH_COUNT       2U
#define DT7_CHANNEL_CENTER     1024
#define DT7_CHANNEL_MIN        364
#define DT7_CHANNEL_MAX        1684
#define DT7_SBUS_CHANNEL_MASK  0x07FFU

/* ── 新鲜度看门狗参数（喂给算法层 Freshness，见 freshness.h）── */
/** 离线判定超时（ms）：距上次合法帧超此值判离线 */
#define DT7_FRESH_TIMEOUT_MS   50U
/** 上线去抖：连续合法帧数（1 = 首个合法帧即在线，保持原行为） */
#define DT7_FRESH_ONLINE_NEED  1U

#define DT7_SWITCH_UP   1U
#define DT7_SWITCH_DOWN 2U
#define DT7_SWITCH_MID  3U

/**
 * @brief  DT7 控制量快照
 * @note   仅含摇杆/拨轮通道与两个拨杆；键鼠字段不参与本工程控制。
 */
typedef struct __packed
{
    struct __packed
    {
        int16_t ch[DT7_CHANNEL_COUNT]; /**< 摇杆×4 + 拨轮×1，值域 [364,1684] */
        uint8_t s[DT7_SWITCH_COUNT];   /**< 左/右拨杆，码值 1=上 3=中 2=下    */
    } rc;
} DT7_ctrl_t;

extern DT7_ctrl_t g_dt7_ctrl;

void DT7_DriverInit(void);
uint8_t DT7_CheckOffline(void);
void DT7_GetSnapshot(DT7_ctrl_t *snapshot);

#endif /* DT7_DRIVER_H */
