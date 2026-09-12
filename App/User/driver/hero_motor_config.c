/**
 ******************************************************************************
 * @file    hero_motor_config.c
 * @brief   英雄机器人电机拓扑配置（每电机 {型号,总线,ID} → 帧组自动推导 + 自检）
 * @note    组织方式：最外层按“模块组件”分组，每个模块只列出其每颗电机的三样：
 *            ① 型号（M3508 / GM6020，决定控制帧字节含义与 ID→帧规则）；
 *            ② 所走 CAN 总线（CAN1）；
 *            ③ 电调 ID（1..8，装车前用 DJI 助手设成此值）。
 *          控制帧组（总线 + 帧 ID）与帧内槽位都由这三样推出，配置者不再手填
 *          帧组，也就不会出现“电机指到错帧”这类静默失效。模块可跨总线
 *          （云台 Yaw 与 Pitch 均在 CAN1）。
 *
 *          HeroMotorConfig_Init 遍历所有电机：按 (总线, 由型号/ID 推出的控制帧)
 *          去重建出实际发送组（同一 (总线,帧) 的多颗电机自动并进一帧），并逐颗
 *          解析出 {发送组下标, 帧内字节偏移} 存入槽位引用表供发送使用。过程中
 *          做三项自检，任一失败即返回 HAL_ERROR（开机即拦，装车前暴露）：
 *            ① ID 范围（1..8）；
 *            ② 发送组数不超过 HERO_TX_GROUP_MAX；
 *            ③ 槽位冲突：同一发送组内同一槽位不允许两颗电机占用。
 *          反馈 ID 的唯一性由接收侧具名宏（bsp_can.h）在编译期保证，不在此列。
 *
 *          通用发送模块 motor_tx 不含任何机器人专属映射；本文件是方案 §5.2
 *          所述“帧组配置表（英雄专属，以 (总线, 帧 ID) 为键）”的落地。
 *          GM6020 采用 0x1FE 电流控制帧。
 * @version 3.0
 * @date    2026-8-12
 ******************************************************************************
 */

#include <hero_motor_config.h>

/* ========================================================================== */
/*                    DJI 控制帧 ID（由“型号 + ID 段”推出）                     */
/* ========================================================================== */

#define HERO_GM6020_CTRL_GROUP_1_ID 0x1FEU /**< GM6020 ID 1..4 电流控制帧       */

/** 发送组数上限：2 条总线 × 至多 3 种控制帧，取宽松上界。 */
#define HERO_TX_GROUP_MAX 1U

/* ========================================================================== */
/*                        DJI 控制帧槽位换算参数（协议相关）                    */
/* ========================================================================== */

#define HERO_ESC_ID_MIN      1U   /**< 合法电调 ID 下限            */
#define HERO_ESC_ID_MAX      4U   /**< GM6020 0x1FE 帧支持的电调 ID 上限 */
#define HERO_ESC_PER_FRAME   4U   /**< 每条控制帧承载的电机数      */
#define HERO_CTRL_SLOT_BYTES 2U   /**< 单颗电机控制量字节数（int16）*/

/* ========================================================================== */
/*                              电机反馈全局实例                                */
/* ========================================================================== */

volatile Motor_Measure_t g_hero_gimbal_motor[HERO_GIMBAL_MOTOR_COUNT];

/* ========================================================================== */
/*                 模块电机表：模块 → 每颗电机 {型号, 总线, ID}                 */
/* ========================================================================== */
/*  数组下标 = 模块内电机编号（0 起）。帧组与槽位不在此填，由下面三样推出，       */
/*  保证 ID / 总线 与帧、槽位天然一致。                                          */

/** 电机所走的 CAN 总线。 */
typedef enum
{
    HERO_BUS_CAN1 = 0
} HeroBusId_t;

typedef struct
{
    HeroBusId_t     bus;        /**< 走哪一路 CAN                        */
    uint8_t         esc_id;     /**< DJI 电调 ID（1..8），装车前设成此值 */
} HeroMotorSpec_t;

/* 底盘 4×M3508：CAN1，ID 1..4（→ 控制帧 0x200） */
/* 云台 2×GM6020：下标 0=Pitch，1=Yaw。
 *   Pitch：CAN1，ID2 → 电流控制帧 0x1FE，slot1，反馈 0x206；
 *   Yaw  ：CAN1，ID1 → 电流控制帧 0x1FE，slot0，反馈 0x205。
 */
static const HeroMotorSpec_t s_gimbal_spec[HERO_GIMBAL_MOTOR_COUNT] = {
    {HERO_BUS_CAN1, 2U},
    {HERO_BUS_CAN1, 1U},
};

/* ========================================================================== */
/*              解析结果：每颗电机 → {发送组下标, 帧内字节偏移}                 */
/* ========================================================================== */
/*  Init 时由上面的 spec 推出并填入；发送侧只读这里，热路径不做任何查找。        */

typedef struct
{
    uint8_t group_index; /**< 所属发送组在 s_tx_groups 中的下标 */
    uint8_t byte_offset; /**< 控制帧内字节偏移（0/2/4/6）       */
    uint8_t valid;       /**< 是否已成功解析（未解析禁止发送）  */
} HeroSlotRef_t;

static HeroSlotRef_t s_gimbal_slot[HERO_GIMBAL_MOTOR_COUNT];

/* 供 Init 统一遍历的模块登记表（spec 输入、slot 输出成对登记）。 */
typedef struct
{
    const HeroMotorSpec_t *specs;
    HeroSlotRef_t         *slots;
    uint8_t                count;
} HeroModule_t;

#define HERO_MODULE_COUNT 1U
static const HeroModule_t s_modules[HERO_MODULE_COUNT] = {
    {s_gimbal_spec,   s_gimbal_slot,   HERO_GIMBAL_MOTOR_COUNT},
};

/* 实际发送组（由所有电机去重推出），及其数量。 */
static MotorTxGroup_t s_tx_groups[HERO_TX_GROUP_MAX];
static uint8_t        s_tx_group_count;

/* ========================================================================== */
/*                              内部辅助                                        */
/* ========================================================================== */

/** 总线枚举 → HAL CAN 句柄。 */
static CAN_HandleTypeDef *Hero_BusHandle(HeroBusId_t bus)
{
    (void)bus;
    return &hcan1;
}

/** 由电调 ID 推控制帧内字节偏移（同帧 4 颗依次占 0/2/4/6）。 */
static uint8_t Hero_SlotByteOffset(uint8_t esc_id)
{
    return (uint8_t)(((esc_id - 1U) % HERO_ESC_PER_FRAME) * HERO_CTRL_SLOT_BYTES);
}

/**
 * @brief  由电机类型 + 电调 ID 推其 DJI 控制帧 ID
 * @note   M3508/M2006：1..4→0x200，5..8→0x1FF；GM6020：1..4→0x1FE，5..7→0x2FE。
 */
static uint32_t Hero_ExpectedCtrlId(uint8_t esc_id)
{
    (void)esc_id;
    return HERO_GM6020_CTRL_GROUP_1_ID;
}

/**
 * @brief  查找 (总线, 帧 ID) 对应的发送组，不存在则新建并初始化
 * @retval 发送组下标；HERO_TX_GROUP_MAX 表示组数溢出或初始化失败
 */
static uint8_t Hero_FindOrAddGroup(CAN_HandleTypeDef *hcan, uint32_t std_id)
{
    uint8_t i;

    for (i = 0U; i < s_tx_group_count; i++)
    {
        if ((s_tx_groups[i].hcan == hcan) && (s_tx_groups[i].std_id == std_id))
        {
            return i;
        }
    }

    if (s_tx_group_count >= HERO_TX_GROUP_MAX)
    {
        return HERO_TX_GROUP_MAX;
    }
    if (MotorTx_Init(&s_tx_groups[s_tx_group_count], hcan, std_id) != HAL_OK)
    {
        return HERO_TX_GROUP_MAX;
    }
    return s_tx_group_count++;
}

/** 把一颗电机的控制量（int16 电流码值）写进其解析出的发送组槽位。 */
static HAL_StatusTypeDef Hero_WriteSlot(const HeroSlotRef_t *ref, int16_t value)
{
    if (ref->valid == 0U)
    {
        return HAL_ERROR;
    }
    return MotorTx_SetSlot(&s_tx_groups[ref->group_index],
                           ref->byte_offset,
                           value);
}

/* ========================================================================== */
/*                              公开接口                                        */
/* ========================================================================== */

HAL_StatusTypeDef HeroMotorConfig_Init(void)
{
    uint8_t used[HERO_TX_GROUP_MAX] = {0}; /* 每组已占槽位位图（bit=槽序）*/
    uint8_t m;
    uint8_t k;

    s_tx_group_count = 0U;

    for (m = 0U; m < HERO_MODULE_COUNT; m++)
    {
        const HeroMotorSpec_t *specs = s_modules[m].specs;
        HeroSlotRef_t *slots = s_modules[m].slots;

        for (k = 0U; k < s_modules[m].count; k++)
        {
            uint8_t esc_id = specs[k].esc_id;
            CAN_HandleTypeDef *hcan;
            uint32_t ctrl_id;
            uint8_t gi;
            uint8_t slot_bit;

            slots[k].valid = 0U;

            /* 自检①：ID 范围 */
            if ((esc_id < HERO_ESC_ID_MIN) || (esc_id > HERO_ESC_ID_MAX))
            {
                return HAL_ERROR;
            }

            hcan = Hero_BusHandle(specs[k].bus);
            ctrl_id = Hero_ExpectedCtrlId(esc_id);

            /* 自检②：并帧去重，组数不得超过上限 */
            gi = Hero_FindOrAddGroup(hcan, ctrl_id);
            if (gi >= HERO_TX_GROUP_MAX)
            {
                return HAL_ERROR;
            }

            /* 自检③：同组同槽不可复用 */
            slot_bit = (uint8_t)(1U << ((esc_id - 1U) % HERO_ESC_PER_FRAME));
            if ((used[gi] & slot_bit) != 0U)
            {
                return HAL_ERROR;
            }
            used[gi] |= slot_bit;

            slots[k].group_index = gi;
            slots[k].byte_offset = Hero_SlotByteOffset(esc_id);
            slots[k].valid = 1U;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef HeroMotorTx_SetGimbalCurrent(uint8_t motor_index,
                                               int16_t current)
{
    if (motor_index >= HERO_GIMBAL_MOTOR_COUNT)
    {
        return HAL_ERROR;
    }
    return Hero_WriteSlot(&s_gimbal_slot[motor_index], current);
}

void HeroMotorTx_ClearAll(void)
{
    uint8_t i;

    for (i = 0U; i < s_tx_group_count; i++)
    {
        MotorTx_Clear(&s_tx_groups[i]);
    }
}

HAL_StatusTypeDef HeroMotorTx_FlushAll(void)
{
    return MotorTx_Flush(s_tx_groups, s_tx_group_count);
}
