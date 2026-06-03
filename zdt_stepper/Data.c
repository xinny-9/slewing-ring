/**
 * @file    Data.c
 * @brief   电机数据读取与状态判断模块实现文件
 * @details 提供从电机驱动器读取状态数据并解析的功能
 */

#include "Data.h"


// 函数声明
void Data_judge(Motor_Control *Motor);


/**
 * @brief   读取电机状态数据函数
 * @param   Motrol  电机控制结构体指针
 * @details 发送读取命令到电机驱动器，接收返回的状态数据并解析
 *          命令格式：[地址][0x3A][校验码]
 *          返回数据格式：[地址][命令码][状态字节][校验码]
 *          状态字节各位含义：
 *          - bit0: 使能状态 (GET_EN)
 *          - bit1: 定点状态 (GET_POINT)
 *          - bit2: 锁定状态 (GET_LOCK)
 */
void Zhang_Reading_Data(Motor_Control *Motor)
{

    uint8_t Tx_Data[3]={0};
    Tx_Data[0]=Motor->Motor_Address;  // 电机地址
    Tx_Data[1]=0x3A;                      // 读取状态命令码
    Tx_Data[2]=0x6B;                      // 固定校验值
    
    HAL_UART_AbortReceive(Motor->Usart);  // 终止之前的接收操作
    __HAL_UART_CLEAR_OREFLAG(Motor->Usart);  // 清除UART溢出标志
    
    HAL_UART_Transmit(Motor->Usart,Tx_Data,3,100);  // 发送读取命令，超时100ms
    
    // 接收4字节状态数据，超时50ms
    if (HAL_UART_Receive(Motor->Usart, Motor->Reading_Data,4,50) == HAL_OK)
    {
        Data_judge(Motor);  // 解析接收到的状态数据
    }
}

/**
 * @brief   电机状态数据解析函数
 * @param   Motrol  电机控制结构体指针
 * @details 解析从电机驱动器接收到的状态数据，更新结构体中的状态标志
 *          通过位与运算检查状态字节的各个位
 */
void Data_judge(Motor_Control *Motor)
{
    // 先重置所有状态标志
    Motor->EN_State = 0;
    Motor->POINT_State = 0;
    Motor->LOCK_State = 0;
    
    // 检查状态字节的bit0（使能状态）
    if(Motor->Reading_Data[2]& GET_EN)
    {
        Motor->EN_State=1;  // 电机已使能
    }   
    // 检查状态字节的bit1（定点状态）
    if(Motor->Reading_Data[2]& GET_POINT)
    {
        Motor->POINT_State=1;  // 电机到达指定位置
    }    
    // 检查状态字节的bit2（锁定状态）
    if(Motor->Reading_Data[2]& GET_LOCK)
    {
        Motor->LOCK_State=1;  // 电机处于锁定状态
    }
}
