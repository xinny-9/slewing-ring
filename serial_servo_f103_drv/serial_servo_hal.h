/**
 * *****************************************************************************
 * @file    serial_servo_hal.h
 * @brief   串口总线舵机 STM32F103C8T6 硬件移植层头文件 (USART1 专用)
 * *****************************************************************************
 */

#ifndef __SERIAL_SERVO_HAL_H
#define __SERIAL_SERVO_HAL_H

#include "stm32f1xx_hal.h"
#include "serial_servo.h"

/* ========================== 舵机总线硬件配置 ========================== */

/* 绑定总线舵机连接的串口句柄 (USART1) */
extern UART_HandleTypeDef huart1;
#define SERIAL_SERVO_UART              huart1

/* 半双工通信模式开关 
 * 1: 使用 STM32 内置单线半双工模式 (只接 PA9/TX 引脚，外部必须接 4.7K~10KΩ 上拉电阻)
 * 0: 使用外置收发方向切换芯片 (双引脚控制)
 */
#define SERIAL_SERVO_USE_SINGLE_WIRE   1

#if !SERIAL_SERVO_USE_SINGLE_WIRE
/* 若为外置芯片，在此处配置具体的使能控制引脚 */
#define SERIAL_SERVO_RX_EN_PORT        GPIOB
#define SERIAL_SERVO_RX_EN_PIN         GPIO_PIN_0
#define SERIAL_SERVO_TX_EN_PORT        GPIOB
#define SERIAL_SERVO_TX_EN_PIN         GPIO_PIN_1
#endif

/* ===================================================================== */

/* 方向动态翻转宏定义 */
#if SERIAL_SERVO_USE_SINGLE_WIRE
#define SERIAL_SERVO_DIR_TX()     HAL_HalfDuplex_EnableTransmitter(&SERIAL_SERVO_UART)
#define SERIAL_SERVO_DIR_RX()     HAL_HalfDuplex_EnableReceiver(&SERIAL_SERVO_UART)
#define SERIAL_SERVO_DIR_IDLE()   HAL_HalfDuplex_EnableReceiver(&SERIAL_SERVO_UART)
#else
#define SERIAL_SERVO_DIR_TX()  do { \
    HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_PORT, SERIAL_SERVO_RX_EN_PIN, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_PORT, SERIAL_SERVO_TX_EN_PIN, GPIO_PIN_RESET); \
} while(0)
#define SERIAL_SERVO_DIR_RX()  do { \
    HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_PORT, SERIAL_SERVO_RX_EN_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_PORT, SERIAL_SERVO_TX_EN_PIN, GPIO_PIN_SET);   \
} while(0)
#define SERIAL_SERVO_DIR_IDLE() do { \
    HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_PORT, SERIAL_SERVO_RX_EN_PIN, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_PORT, SERIAL_SERVO_TX_EN_PIN, GPIO_PIN_SET);   \
} while(0)
#endif

/* 声明全局唯一的总线舵机控制器对象 */
extern SerialServoControllerTypeDef g_serial_servo_controller;

/* --- 外部可调用移植接口 --- */
void Serial_Servo_HAL_Init(void);
void Serial_Servo_RxCallback(UART_HandleTypeDef *huart);

#endif /* __SERIAL_SERVO_HAL_H */
