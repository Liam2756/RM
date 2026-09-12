/**
 ******************************************************************************
 * @file    vision_test.h
 * @brief   视觉链路自注入测试模块（临时件，验证完删除）
 * @note    ========================= VISION LINK TEST (deletable) =========================
 *          用途：没有 UART 硬件喂数据时，在固件内部拼一帧合法视觉数据，
 *                周期性地"假装 UART6 收到了这帧"，直接喂给真实解析入口
 *                Vision_UART_DataReadyCallback()。走的软件路径与真实视觉完全一致：
 *                组帧同步 → CRC16 校验 → xyz 解析 → GetNewTarget → 坐标换算 → 云台。
 *
 *          本版：注入【线性缓变】坐标——X(前向距离)固定，Y/Z 在 START↔END 之间
 *                匀速线性变化（可选折返往复），使 yaw/pitch 连续慢扫，便于观察云台移动。
 *
 *          独立性：本模块只调用公开接口（Vision_UART_DataReadyCallback / CRC16_Append），
 *                  不碰任何原模块内部，不改 gimbal / vision_uart 一行。
 *
 *          彻底删除步骤（4 处，均带 "VISION LINK TEST (deletable)" 注释）：
 *            1. 删除 vision_test.c / vision_test.h 两个文件
 *            2. CMakeLists.txt：移除 vision_test.c 那一行
 *            3. app.c：移除 #include <vision_test.h>
 *            4. app.c：移除 VisionTest_Init() 与 VisionTest_Tick() 两处调用
 *          （或临时停用：把下方 VISION_TEST_INJECT_ENABLE 置 0。）
 *
 *          安全说明：注入本身【不驱动云台】，只往视觉目标缓存塞数据。云台是否动作
 *                    仍由原程序的解保(左拨杆) + 视觉模式(右拨杆 s[1]=UP)决定。
 *                    验云台前先手动把云台回中放平，再解保 + s[1]=UP。
 * @version 2.0
 * @date    2026-9-8
 ******************************************************************************
 */

#ifndef VISION_TEST_H
#define VISION_TEST_H

/* ========================================================================== */
/*                              配置区                                          */
/* ========================================================================== */

/** 主开关：1=启用自注入；0=空实现（零 footprint，等价于未引入本模块）。*/
#define VISION_TEST_INJECT_ENABLE   0

/** 前向距离（mm），固定不变。坐标 → 角度换算（见 app.c）：
 *    horizontal = sqrt(x^2 + y^2)
 *    yaw   = -atan2(y, x)      （y 增大 → yaw 变负，云台偏一侧）
 *    pitch =  atan2(z, horizontal)  （z 增大 → 上仰）
 *  X 越大，同样的 Y/Z 位移对应的角度越小（扫描越平缓）。*/
#define VISION_TEST_X_MM            600.0f

/** Y 线性扫描区间（mm）：控制 yaw。START→END 匀速变化。
 *  默认 -100→100：yaw 约 +9.5° → -9.5°（对称扫过中心）。*/
#define VISION_TEST_Y_START_MM      (-1000.0f)
#define VISION_TEST_Y_END_MM        (50.0f)

/** Z 线性扫描区间（mm）：控制 pitch。START→END 匀速变化。
 *  默认 -50→50：pitch 约 -4.7° → +4.7°。
 *  若只想扫单轴，把另一轴的 START 与 END 设成相等即可（该轴保持不动）。*/
#define VISION_TEST_Z_START_MM      (-500.0f)
#define VISION_TEST_Z_END_MM        (500.0f)

/** 单程扫描时长（ms）：从 START 线性走到 END 所用时间。越大越慢。
 *  默认 3000ms = 3s 走完全程（约 3.8°/s，缓慢可观察）。*/
#define VISION_TEST_SWEEP_MS        600U

/** 折返：1=到 END 后反向匀速走回 START，如此三角波持续往复；
 *        0=到 END 后保持不变（走一趟就停）。*/
#define VISION_TEST_PINGPONG        1

/** 注入帧周期（ms）：每隔多少毫秒喂一帧，模拟视觉帧率。默认 10ms = 100Hz。*/
#define VISION_TEST_PERIOD_MS       10U

/* ========================================================================== */
/*                              接口                                            */
/* ========================================================================== */

/**
 * @brief  初始化（复位扫描相位）。在 Vision_UART_Init() 之后、进入调度前调用一次。
 */
void VisionTest_Init(void);

/**
 * @brief  周期钩子（放 1ms 控制任务里，须在 Vision_UART_GetNewTarget() 之前）。
 * @note   每 1ms 推进扫描相位；每满 VISION_TEST_PERIOD_MS 按当前相位拼帧并注入。
 */
void VisionTest_Tick(void);

#endif /* VISION_TEST_H */
