/**
 ******************************************************************************
 * @file    dt7_driver.c
 * @brief   DT7/DR16 遥控器 SBUS 数据解析实现（设备驱动层）
 * @note    校验分级：结构（空指针/帧长）+ 内容（拨杆码值）不合法即丢帧、
 *          保留上一帧且不刷新在线时间；通道越界只钳位、不否决整帧。
 *          鼠标/键盘字段不参与本工程控制。
 * @version 2.2
 * @date    2026-8-16
 * @note    v2.1：① 通道由"越界否决整帧"改为 DT7_ClampChannels 钳位（C-2），
 *          避免单通道瞬时贴边丢掉整帧遥控输入；② 删除无人使用的鼠标/键盘
 *          解析与清零（死代码，C-3）；③ 补全各函数注释。
 * @note    v2.2：在线/离线判定改用算法层 Freshness 看门狗（G-1），仅合法帧
 *          经 Freshness_FeedGood 刷新在线时间，超时判离线并复位为中立态。
 ******************************************************************************
 */

#include <dt7_driver.h>
#include <freshness.h>

DT7_ctrl_t g_dt7_ctrl;

/* 在线/离线判定委托给算法层 Freshness 看门狗；参数取 DT7_FRESH_*（见头文件）。 */
static Freshness_t s_dt7_fresh;
static const Freshness_Config_t s_dt7_fresh_cfg = {
    .timeout_ms  = DT7_FRESH_TIMEOUT_MS,
    .online_need = DT7_FRESH_ONLINE_NEED,
};

/**
 * @brief  将控制量复位为安全中立态（通道居中、拨杆清零）
 * @param  control  待复位的控制结构
 * @note   离线判定时调用，防止控制层读到残余的非中立摇杆/拨杆数据。
 */
static void DT7_ResetData(DT7_ctrl_t *control)
{
    uint8_t i;

    for (i = 0U; i < DT7_CHANNEL_COUNT; i++)
    {
        control->rc.ch[i] = DT7_CHANNEL_CENTER;
    }
    control->rc.s[0] = 0U;
    control->rc.s[1] = 0U;
}

/**
 * @brief  从 18 字节 SBUS 帧解析摇杆通道与拨杆到控制结构
 * @param  buf      指向 SBUS 原始帧
 * @param  control  输出控制量（仅填 rc，不含键鼠）
 * @note   11 位通道跨字节位打包，需逐字节移位组合后掩码截取：
 *         ch[0..3] 来自 buf[0..5]，ch[4]（拨轮）来自 buf[16..17]；
 *         两个 2 位拨杆在 buf[5] 高 4 位；buf[6..15] 的鼠标/键盘不参与控制，
 *         此处不解析。
 */
static void DT7_ParseFrame(const uint8_t *buf, DT7_ctrl_t *control)
{
    control->rc.ch[0] =
        (int16_t)((buf[0] | ((uint16_t)buf[1] << 8)) &
                  DT7_SBUS_CHANNEL_MASK);
    control->rc.ch[1] =
        (int16_t)(((uint16_t)buf[1] >> 3 |
                  ((uint16_t)buf[2] << 5)) &
                  DT7_SBUS_CHANNEL_MASK);
    control->rc.ch[2] =
        (int16_t)(((uint16_t)buf[2] >> 6 |
                  ((uint16_t)buf[3] << 2) |
                  ((uint16_t)buf[4] << 10)) &
                  DT7_SBUS_CHANNEL_MASK);
    control->rc.ch[3] =
        (int16_t)(((uint16_t)buf[4] >> 1 |
                  ((uint16_t)buf[5] << 7)) &
                  DT7_SBUS_CHANNEL_MASK);
    control->rc.ch[4] =
        (int16_t)((buf[16] | ((uint16_t)buf[17] << 8)) &
                  DT7_SBUS_CHANNEL_MASK);

    control->rc.s[0] = (uint8_t)((buf[5] >> 6) & 0x03U);
    control->rc.s[1] = (uint8_t)((buf[5] >> 4) & 0x03U);
}

/**
 * @brief  判断拨杆码值是否为合法档位
 * @param  value  2 位拨杆码值
 * @retval 1 合法（上/中/下），0 非法（含 0 = 健康遥控器不会发送的未定义值）
 */
static uint8_t DT7_IsSwitchValid(uint8_t value)
{
    return ((value == DT7_SWITCH_UP) ||
            (value == DT7_SWITCH_DOWN) ||
            (value == DT7_SWITCH_MID)) ? 1U : 0U;
}

/**
 * @brief  将五个通道钳位到合法值域 [DT7_CHANNEL_MIN, DT7_CHANNEL_MAX]
 * @param  control  待钳位的控制结构
 * @note   通道略超边界视为正常抖动而非坏帧，直接钳回区间、不否决整帧，
 *         避免单通道瞬时越界丢掉整帧遥控输入。
 */
static void DT7_ClampChannels(DT7_ctrl_t *control)
{
    uint8_t i;

    for (i = 0U; i < DT7_CHANNEL_COUNT; i++)
    {
        if (control->rc.ch[i] < DT7_CHANNEL_MIN)
        {
            control->rc.ch[i] = DT7_CHANNEL_MIN;
        }
        else if (control->rc.ch[i] > DT7_CHANNEL_MAX)
        {
            control->rc.ch[i] = DT7_CHANNEL_MAX;
        }
        else
        {
            /* 在值域内，保持不变 */
        }
    }
}

/**
 * @brief  初始化 DT7 驱动：控制量置中立、看门狗复位（初始判离线）
 */
void DT7_DriverInit(void)
{
    DT7_ResetData(&g_dt7_ctrl);
    Freshness_Init(&s_dt7_fresh, &s_dt7_fresh_cfg);
}

/**
 * @brief  查询遥控器是否离线；离线时复位控制量为中立
 * @retval 1 离线，0 在线
 * @note   离线判定委托给 Freshness 看门狗（超时 DT7_FRESH_TIMEOUT_MS）。
 *         查询与复位在临界区内完成，避免与接收回调竞争；离线即把控制量
 *         复位为安全中立态，防止控制层读到残余的非中立摇杆/拨杆。
 */
uint8_t DT7_CheckOffline(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t primask = __get_PRIMASK();
    uint8_t offline;

    __disable_irq();
    offline = (Freshness_IsOnline(&s_dt7_fresh, now_tick) == 0U) ? 1U : 0U;
    if (offline != 0U)
    {
        DT7_ResetData(&g_dt7_ctrl);
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return offline;
}

/**
 * @brief  原子拷贝当前控制量快照，供控制/应用层读取
 * @param  snapshot  输出缓冲；为空指针则不操作
 * @note   临界区内整体拷贝，避免读到接收回调更新一半的撕裂数据。
 */
void DT7_GetSnapshot(DT7_ctrl_t *snapshot)
{
    uint32_t primask;

    if (snapshot == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = g_dt7_ctrl;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/**
 * @brief  覆盖 bsp_uart.c 弱回调，接入 DT7 SBUS 解析与提交
 * @param  buf   指向收到的一帧数据
 * @param  size  收到的字节数
 * @note   由 HAL_UARTEx_RxEventCallback 在数据就绪后调用。
 *         校验两级：① 结构（空指针/帧长）② 内容（拨杆码值）；不合法即丢帧、
 *         保留上一帧且不刷新在线时间。通道越界只钳位不否决。提交在临界区内
 *         整体拷贝，避免控制环 ISR 读到撕裂数据。
 */
void BSP_UART_DT7_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    DT7_ctrl_t parsed;
    uint32_t primask;

    if ((buf == 0) || (size != DT7_FRAME_LENGTH))
    {
        return; /* 结构错：丢帧 */
    }

    DT7_ParseFrame(buf, &parsed);

    /* 拨杆非码值 = 损坏/错位帧：丢帧、保上一帧、不刷新在线时间 */
    if ((DT7_IsSwitchValid(parsed.rc.s[0]) == 0U) ||
        (DT7_IsSwitchValid(parsed.rc.s[1]) == 0U))
    {
        return;
    }

    DT7_ClampChannels(&parsed); /* 通道贴边：钳位，不否决 */

    primask = __get_PRIMASK();
    __disable_irq();
    g_dt7_ctrl = parsed;
    Freshness_FeedGood(&s_dt7_fresh, HAL_GetTick()); /* 仅合法帧刷新在线时间 */
    if (primask == 0U)
    {
        __enable_irq();
    }
}
