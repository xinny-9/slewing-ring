/**
 * @file    Data.h
 * @brief   电机数据读取与状态判断模块头文件
 * @details 定义电机状态读取相关的宏定义和函数接口
 *          支持读取电机的使能状态、定点状态和锁定状态
 */

#ifndef __Data_H
#define __Data_H

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "CONTROL.h"

/**
 * @defgroup 状态标志位宏定义
 * @brief    定义电机状态字节的各个位掩码
 * @details  用于从状态字节中提取特定的状态信息
 * @{
 */
#define GET_EN 0x01    /**< 使能状态标志位（bit0）：1-已使能，0-已失能 */
#define GET_POINT 0x02 /**< 定点状态标志位（bit1）：1-到达指定位置，0-未到达 */
#define GET_LOCK 0x04  /**< 锁定状态标志位（bit2）：1-锁定状态，0-未锁定 */
/** @} */

/**
 * @defgroup 数据读取函数
 * @brief    提供电机状态数据的读取和解析功能
 * @{
 */
void Zhang_Reading_Data(Motor_Control *Motor);
/**
 * @brief   电机状态数据解析函数
 * @param   Motrol  电机控制结构体指针
 * @details 解析从电机驱动器接收到的状态数据，更新结构体中的状态标志
 *          通过位与运算检查状态字节的各个位
 */
void Data_judge(Motor_Control *Motor);
/** @} */


#endif
