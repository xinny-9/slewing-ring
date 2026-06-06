/**
 * *****************************************************************************
 * @file    serial_servo_hal.c
 * @brief   串口总线舵机 F103 硬件层收发驱动实现 (自适应半双工与电回波消除)
 * *****************************************************************************
 */

#include "serial_servo_hal.h"
#include <string.h>

/* 全局控制器实例 */
SerialServoControllerTypeDef g_serial_servo_controller;

/* 用于串口 1 中断接收的 1 字节数据缓冲 */
static uint8_t g_servo_rx_temp_byte = 0;

/**
 * @brief  底层物理收发具体实现函数 (包含方向快速翻转与电回波过滤)
 */
static int serial_write_and_read_impl(SerialServoControllerTypeDef *self, SerialServoCmdTypeDef *frame, bool tx_only)
{
    // 1. 切换物理层为：发送模式 (依靠 STM32 硬件半双工自动将接收器 RE 禁用，因此发送时在单线上绝不会产生回波触发中断)
    SERIAL_SERVO_DIR_TX();
    
    // 计算即将发送的数据包总长度 (Length + 帧头2字节 + 校验和1字节)
    uint32_t send_len = frame->elements.length + 3;
    
    // 2. 阻塞发送数据帧至总线
    HAL_UART_Transmit(&SERIAL_SERVO_UART, (uint8_t*)frame, send_len, 100);
    
    // ⚠️核心技术：等待发送移位寄存器彻底排空 (防止提前切换接收导致丢尾字节)
    while(__HAL_UART_GET_FLAG(&SERIAL_SERVO_UART, UART_FLAG_TC) == RESET);
    
    // 3. 决定是否需要等待应答
    if (tx_only) {
        // 单向控制指令，直接切回空闲接收状态 (硬件自动重新使能接收器 RE)
        SERIAL_SERVO_DIR_IDLE();
        return 0;
    }
    
    // 重置控制器接收状态机与完成标志
    self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
    self->rx_completed = false;
    
    // 4. 双向指令：极速切换物理层为：接收模式 (硬件极速重新使能接收器 RE，耗时小于1微秒，彻底杜绝了因重新调用 IT 函数导致的 ORE 溢出或时序不赶趟)
    SERIAL_SERVO_DIR_RX();
    
    // ⚠️核心技术：强行清空硬件接收可能由于电噪声激发的 ORE 错误，确保接收中断顺畅触发
    __HAL_UART_CLEAR_OREFLAG(&SERIAL_SERVO_UART);
    
    // 5. 软定时器超时机制 (默认 8ms 防卡死，等待直到后台常驻中断将 rx_completed 置位)
    uint32_t start_tick = HAL_GetTick();
    while (!self->rx_completed) {
        if (HAL_GetTick() - start_tick > self->proc_timeout) {
            // 超时退出，强制恢复空闲，并清理可能残留的半包溢出错误
            SERIAL_SERVO_DIR_IDLE();
            __HAL_UART_CLEAR_OREFLAG(&SERIAL_SERVO_UART);
            return -1; 
        }
    }
    
    // 成功接收到并校验通过了舵机回复的完整帧，恢复空闲模式
    SERIAL_SERVO_DIR_IDLE();
    return 0;
}

/**
 * @brief  初始化总线舵机物理层，开启首次中断接收监听
 */
void Serial_Servo_HAL_Init(void)
{
    // 初始化控制器管理对象参数
    serial_servo_controller_object_init(&g_serial_servo_controller);
    
    // 挂载收发实现函数指针
    g_serial_servo_controller.serial_write_and_read = serial_write_and_read_impl;
    g_serial_servo_controller.proc_timeout = 8; // 8ms 超时上限
    
    // 初始化串口方向为默认监听接收状态
    SERIAL_SERVO_DIR_IDLE();
    
    // ⚠️ 极其重要：上电启动首次常驻 1 字节中断接收，依靠中断回调内的自挂载维持后台常驻监听
    HAL_UART_Receive_IT(&SERIAL_SERVO_UART, &g_servo_rx_temp_byte, 1);
}

/**
 * @brief  标准接收回调处理器
 */
void Serial_Servo_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == SERIAL_SERVO_UART.Instance) {
        // 送入协议解析状态机
        serial_servo_rx_handler(&g_serial_servo_controller, g_servo_rx_temp_byte);
        
        // 维持下一次的 1 字节中断监听
        HAL_UART_Receive_IT(&SERIAL_SERVO_UART, &g_servo_rx_temp_byte, 1);
    }
}
