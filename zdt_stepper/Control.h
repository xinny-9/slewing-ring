/**
 * @file    Control.h
 * @brief   步进电机控制模块头文件
 * @details 定义电机控制相关的数据结构、宏定义和函数接口
 *          支持电机的使能控制、速度控制、位置控制、细分设置和地址修改
 */

#ifndef __CONTROL_H 
#define __CONTROL_H 


#include "main.h" 
#include "usart.h" 
#include "gpio.h" 


#define UP 1            /**< 电机转动方向：1-顺时针，0-逆时针 */
#define DOWN 0
#define Blind_Time 500  /**< 电机定位盲区时间（毫秒），防止频繁触发 */



/**
 * @defgroup 位置预定义宏
 * @brief    定义电机转动的预设位置（以步数为单位）
 * @{
 */
#define Location_0 0        /**< 原点位置 */
#define Location_1_4 224444 /**< 1/4圈位置 */
#define Location_1_2 448889 /**< 1/2圈位置 */
#define Location_3_4 673333 /**< 3/4圈位置 */
#define Location_1   897778 /**< 1圈位置 */
/** @} */


/**
 * @defgroup 使能状态宏
 * @brief    定义电机的使能状态
 * @{
 */
#define Enable 1   /**< 使能电机 */
#define Disable 0  /**< 失能电机 */
/** @} */

/**
 * @brief   电机控制结构体
 * @details 保存电机的配置参数和运行状态信息
 */
typedef struct
{
    uint8_t Motor_Address;      /**< 电机通信地址（1-255） */
    UART_HandleTypeDef *Usart;   /**< UART串口句柄指针，用于与电机驱动器通信 */  
    uint8_t State;               /**< 电机位置状态：0-原点，1-1/4圈，2-1/2圈，3-3/4圈，4-1圈 */
    uint8_t EN_State;            /**< 电机使能状态：0-失能，1-使能 */
    uint8_t POINT_State;         /**< 定点状态（具体含义由应用定义） */
    uint8_t LOCK_State;          /**< 锁定状态（具体含义由应用定义） */
    
    
    uint32_t Current_Time;
    uint32_t Last_Time;
    uint32_t Motor_Blind_Time;
    uint8_t Reading_Data[8];     /**< 读取数据缓冲区，用于存储从电机读取的数据 */
    
}Motor_Control;

/**
 * @defgroup 电机控制函数
 * @brief    提供电机的各种控制功能
 * @{
 */

void Motor_Position(Motor_Control *Motor,uint8_t Direction,uint16_t Speed,uint8_t Accelerate,uint32_t Where);


/**
 * @brief   电机初始化函数
 * @param   Motor         电机控制结构体指针，用于存储电机运行状态和配置信息
 * @param   Motor_Address 电机通信地址（1-255），用于多电机系统中的地址识别
 * @details 初始化电机控制结构体的所有成员变量，包括地址、UART句柄和状态标志位
 *          必须在调用任何电机控制函数之前先调用此函数
 */
void Control_Init(Motor_Control *Motor,UART_HandleTypeDef *huart,uint8_t Motor_Address);

/**
 * @brief   电机使能控制函数
 * @param   Motor   电机控制结构体指针
 * @param   ENABLE  使能状态：1-使能电机，0-失能电机
 * @details 发送使能命令到电机驱动器，控制电机的使能状态
 *          使能后电机才能响应运动控制命令
 */
void Control_En(Motor_Control *Motor,uint8_t ENABLE);

/**
 * @brief   电机速度控制函数
 * @param   Motor      电机控制结构体指针
 * @param   Direction  旋转方向：0-顺时针，1-逆时针
 * @param   Speed      速度值（0-3000），超过3000会被限制为3000
 * @param   Accelerate 加速度值，控制电机启动和停止的加减速过程
 * @details 发送速度控制命令，使电机以指定速度和加速度持续旋转
 *          速度值越大，电机转速越快
 */
void Control_Speed(Motor_Control *Motor,uint8_t Direction,uint16_t Speed,uint8_t Accelerate);
/**
 * @brief   电机细分设置函数
 * @param   Motor      电机控制结构体指针
 * @param   Subdivide  细分值，决定步进电机的每转步数
 * @details 设置电机驱动器的细分参数，影响电机精度和平滑度
 *          细分值越大，电机运行越平滑，但最高速度可能降低
 */
void Control_Subdivide(Motor_Control *Motor,uint8_t Subdivide);

/**
 * @brief   电机地址修改函数
 * @param   Motor   电机控制结构体指针
 * @param   Address 新地址值（1-255），地址0会被自动改为1
 */
void Control_Address(Motor_Control *Motor,uint8_t Address);

/**
 * @brief   电机数据周期性读取函数
 * @param   Motor        电机控制结构体指针
 * @param   Reading_Time 数据读取周期（毫秒），推荐值为100ms
 * @note    推荐在主函数中调用，Reading_Time 推荐设置为100ms
 */
void Motor_Reading(Motor_Control *Motor,uint32_t Reading_Time);

/**
 * @brief   电机定位控制函数（带盲区时间保护）
 * @param   Motor      电机控制结构体指针
 * @param   Speed      运行速度（0-3000）
 * @param   Accelerate 加速度值
 * @param   Where      目标位置，可使用预定义位置宏或自定义位置值
 */
uint8_t Motor_Want_Position(Motor_Control *Motor,uint16_t Speed,uint8_t Accelerate,uint32_t Where);

#endif
