/**
 ******************************************************************************
 * @file    bsp_flash.h
 * @brief   片内 Flash 读写驱动头文件（BSP 层）
 * @note    层级说明：
 *            本文件属于【BSP 层】，封装 STM32F4 片内 Flash 操作。
 *            适用于 STM32F407/STM32F427 等 F4 系列（双 Bank，共 12 个扇区）。
 *
 *          对外接口：
 *            flash_read()               → memcpy 直接读（Flash 可字节寻址）
 *            flash_erase_address()      → 按扇区擦除（须先擦后写）
 *            flash_write_single_address()→ 按字（32bit）编程写入
 *            get_sector()               → 地址→扇区号映射
 *            get_next_flash_address()   → 获取当前扇区的末尾地址
 *
 *          Flash 地址空间说明（STM32F407，1MB）：
 *            Sector 0~3  : 16KB × 4
 *            Sector 4    : 64KB × 1
 *            Sector 5~11 : 128KB × 7
 *            FLASH_END_ADDR = 0x08100000（1MB 边界）
 *
 *          使用注意：
 *            1. 写之前必须先擦除对应扇区（擦除以扇区为单位，不可部分擦除）
 *            2. HAL_FLASH_Unlock/Lock 在读写函数内部已成对调用
 *            3. flash_write_single_address 的 static 局部变量非线程安全，
 *               裸机单任务环境下使用无问题。
 *
 *          TODO: get_sector 函数名有拼写错误（应为 get_sector），
 *                暂保留以兼容已有调用，后续统一重命名。
 *
 * @version 1.0
 * @date    2022-9-3
 * @author  DJI
 ******************************************************************************
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H
#include "struct_typedef.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base address of Sector 7, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* Base address of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* Base address of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */

#define FLASH_END_ADDR ((uint32_t)0x08100000)       /* Base address of Sector 23, 128 Kbytes */


#define ADDR_FLASH_SECTOR_12 ((uint32_t)0x08100000) /* Base address of Sector 12, 16 Kbytes  */
#define ADDR_FLASH_SECTOR_13 ((uint32_t)0x08104000) /* Base address of Sector 13, 16 Kbytes  */
#define ADDR_FLASH_SECTOR_14 ((uint32_t)0x08108000) /* Base address of Sector 14, 16 Kbytes  */
#define ADDR_FLASH_SECTOR_15 ((uint32_t)0x0810C000) /* Base address of Sector 15, 16 Kbytes  */
#define ADDR_FLASH_SECTOR_16 ((uint32_t)0x08110000) /* Base address of Sector 16, 64 Kbytes  */
#define ADDR_FLASH_SECTOR_17 ((uint32_t)0x08120000) /* Base address of Sector 17, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18 ((uint32_t)0x08140000) /* Base address of Sector 18, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19 ((uint32_t)0x08160000) /* Base address of Sector 19, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20 ((uint32_t)0x08180000) /* Base address of Sector 20, 128 Kbytes */
#define ADDR_FLASH_SECTOR_21 ((uint32_t)0x081A0000) /* Base address of Sector 21, 128 Kbytes */
#define ADDR_FLASH_SECTOR_22 ((uint32_t)0x081C0000) /* Base address of Sector 22, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23 ((uint32_t)0x081E0000) /* Base address of Sector 23, 128 Kbytes */



/**
  * @brief          erase flash
  * @param[in]      address: flash address
  * @param[in]      len: page num
  * @retval         none
  */
extern void flash_erase_address(uint32_t address, uint16_t len);

/**
  * @brief          write data to one page of flash
  * @param[in]      start_address: flash address
  * @param[in]      buf: data point
  * @param[in]      len: data num
  * @retval         success 0, fail -1
  */
extern int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);


/**
  * @brief          write data to some pages of flash
  * @param[in]      start_address: flash start address
  * @param[in]      end_address: flash end address
  * @param[in]      buf: data point
  * @param[in]      len: data num
  * @retval         success 0, fail -1
  */
extern int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);

/**
  * @brief          read data for flash
  * @param[in]      address: flash address
  * @param[out]     buf: data point
  * @param[in]      len: data num
  * @retval         none
  */
extern void flash_read(uint32_t address, uint32_t *buf, uint32_t len);

/**
  * @brief          get the sector number of flash
  * @param[in]      address: flash address
  * @retval         sector number
  */
extern uint32_t get_sector(uint32_t address);

/**
  * @brief          get the next page flash address
  * @param[in]      address: flash address
  * @retval         next page flash address
  */
extern uint32_t get_next_flash_address(uint32_t address);
#endif
