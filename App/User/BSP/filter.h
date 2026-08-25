/**
 ******************************************************************************
 * @file    filter.h
 * @brief   双 CAN 过滤器和接收中断初始化接口（BSP 层）
 * @version 2.0
 * @date    2026-8-11
 ******************************************************************************
 */

#ifndef FILTER_H
#define FILTER_H

#include <can.h>

HAL_StatusTypeDef can_filter_init(void);

#endif /* FILTER_H */
