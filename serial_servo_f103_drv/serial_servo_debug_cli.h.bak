/**
 * *****************************************************************************
 * @file    serial_servo_debug_cli.h
 * @brief   串口总线舵机 STM32 裸机调试命令行控制台头文件 (USART3 专用)
 * *****************************************************************************
 */

#ifndef __SERIAL_SERVO_DEBUG_CLI_H
#define __SERIAL_SERVO_DEBUG_CLI_H

#include "stm32f1xx_hal.h"
#include "serial_servo_hal.h"

/* ========================== 调试交互串口配置 ========================== */

/* 绑定连接 PC 串口助手的调试串口 (USART3) */
extern UART_HandleTypeDef huart3;
#define DEBUG_CLI_UART          huart3

/* ===================================================================== */

void Debug_CLI_Init(void);
void Debug_CLI_RxCallback(UART_HandleTypeDef *huart);
void Debug_CLI_Process(void);

#endif /* __SERIAL_SERVO_DEBUG_CLI_H */
