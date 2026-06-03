/**
 * *****************************************************************************
 * @file    serial_servo_debug_cli.c
 * @brief   交互控制台指令捕获、解析状态机与 API 映射映射源文件 (USART3 优化版)
 * *****************************************************************************
 */

#include "serial_servo_debug_cli.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Control.h"             // 引入步进电机控制头文件

#define CLI_RX_LEN 64
static uint8_t g_cli_rx_buf[CLI_RX_LEN];
static uint8_t g_cli_rx_index = 0;
static volatile bool g_cli_frame_ready = false;
extern Motor_Control my_motor;   // 声明外部的步进电机控制句柄
/* 串口 3 异步中断单字节接收缓冲区 */
static uint8_t g_cli_rx_temp_byte = 0;

/**
 * @brief  初始化调试命令行控制台并开启调试串口 3 接收中断
 */
void Debug_CLI_Init(void)
{
    g_cli_rx_index = 0;
    g_cli_frame_ready = false;
    
    // 强行清除 ORE 挂起标志，防止上电产生溢出死锁
    __HAL_UART_CLEAR_OREFLAG(&DEBUG_CLI_UART);
    
    // 开启调试串口 3 异步 1 字节中断接收
    HAL_UART_Receive_IT(&DEBUG_CLI_UART, &g_cli_rx_temp_byte, 1);
    
    // 向上位机串口助手输出操作引导菜单
    printf("\r\n==================================================\r\n");
    printf("   ⚙️ 串口总线舵机 STM32F103 调试 CLI 控制台就绪 (USART3)\r\n");
    printf("   使用说明: 在串口助手输入以下指令 (需勾选发送新行):\r\n");
    printf("     1. pos <id> <pos> <time>  -> 控制转动到目标位置 (0~1000)\r\n");
    printf("     2. read <id>             -> 一键回读位置、电压和温度\r\n");
    printf("     3. stop <id>             -> 紧急停止锁死\r\n");
    printf("     4. free <id>             -> 释放力矩 (手动教学示教)\r\n");
    printf("     5. lock <id>             -> 重新上电力矩锁死\r\n");
    printf("==================================================\r\n\r\n");
    printf("     6. motor_pos <speed> <acc> <where> -> 步进电机位置控制\r\n");

}

/**
 * @brief  调试串口单字节中断捕获处理器
 */
static void Debug_UART_RxHandler(uint8_t byte)
{
    // ORE 保护：防止字符溢出引起串口死机
    if (__HAL_UART_GET_FLAG(&DEBUG_CLI_UART, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&DEBUG_CLI_UART);
    }

    // 捕获回车或换行作为行结束符
    if (byte == '\n' || byte == '\r') {
        if (g_cli_rx_index > 0) {
            g_cli_rx_buf[g_cli_rx_index] = '\0';
            g_cli_frame_ready = true;
        }
    } else {
        if (g_cli_rx_index < CLI_RX_LEN - 1) {
            g_cli_rx_buf[g_cli_rx_index++] = byte;
        } else {
            g_cli_rx_index = 0; // 溢出防护
        }
    }
}

/**
 * @brief  挂载在标准接收完成回调中的分发接口
 */
void Debug_CLI_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DEBUG_CLI_UART.Instance) {
        // 捕获数据
        Debug_UART_RxHandler(g_cli_rx_temp_byte);
        
        // 极其重要：继续保持下一次 1 字节监听
        HAL_UART_Receive_IT(&DEBUG_CLI_UART, &g_cli_rx_temp_byte, 1);
    }
}

/**
 * @brief  主循环调用命令解析与舵机控制 API 映射器
 */
void Debug_CLI_Process(void)
{
    if (!g_cli_frame_ready) {
        return;
    }
    
    // 使用 strtok 切割传入的 ASCII 命令字符串
    char *cmd = strtok((char*)g_cli_rx_buf, " ");
    if (cmd != NULL) {
        if (strcmp(cmd, "pos") == 0) {
            char *p1 = strtok(NULL, " ");
            char *p2 = strtok(NULL, " ");
            char *p3 = strtok(NULL, " ");
            
            if (p1 && p2 && p3) {
                int id = atoi(p1);
                int pos = atoi(p2);
                int dur = atoi(p3);
                // 直接控制真实的舵机控制器对象
                serial_servo_set_position(&g_serial_servo_controller, id, pos, dur);
                printf(">> [CLI执行]: 控制舵机 %d 运动至位置 %d, 耗时 %dms\r\n", id, pos, dur);
            } else {
                printf(">> 参数错误! 格式应为: pos <id> <pos> <time>\r\n");
            }
        }
        else if (strcmp(cmd, "read") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                int id = atoi(p1);
                int16_t pos = 0;
                uint16_t vin = 0;
                uint8_t temp = 0;
                
                printf(">> [CLI执行]: 正在从总线回读 舵机 %d 实时参数...\r\n", id);
                
                // 顺序执行回读，每次动作之间预留 40ms 物理通道静默间歇，以保障抗电磁干扰度
                int r1 = serial_servo_read_position(&g_serial_servo_controller, id, &pos);
                HAL_Delay(40);
                int r2 = serial_servo_read_vin(&g_serial_servo_controller, id, &vin);
                HAL_Delay(40);
                int r3 = serial_servo_read_temp(&g_serial_servo_controller, id, &temp);
                
                if (r1 == 0 && r2 == 0 && r3 == 0) {
                    printf(">> [回读成功] 实时位置: %4d | 供电电压: %.2f V | 芯片温度: %d ℃\r\n", pos, vin/1000.0f, temp);
                } else {
                    printf(">> [回读失败] 舵机未响应！请排查供电或ID。(r_pos:%d, r_vin:%d, r_temp:%d)\r\n", r1, r2, r3);
                }
            } else {
                printf(">> 参数错误! 格式应为: read <id>\r\n");
            }
        }
        else if (strcmp(cmd, "stop") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                int id = atoi(p1);
                serial_servo_stop(&g_serial_servo_controller, id);
                printf(">> [CLI执行]: 紧急制动 舵机 %d\r\n", id);
            } else {
                printf(">> 参数错误! 格式应为: stop <id>\r\n");
            }
        }
        else if (strcmp(cmd, "free") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                int id = atoi(p1);
                serial_servo_load_unload(&g_serial_servo_controller, id, 0);
                printf(">> [CLI执行]: 释放 舵机 %d 力矩 (进入手动示教模式)\r\n", id);
            } else {
                printf(">> 参数错误! 格式应为: free <id>\r\n");
            }
        }
        else if (strcmp(cmd, "lock") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                int id = atoi(p1);
                serial_servo_load_unload(&g_serial_servo_controller, id, 1);
                printf(">> [CLI执行]: 锁死 舵机 %d 力矩\r\n", id);
            } else {
                printf(">> 参数错误! 格式应为: lock <id>\r\n");
            }
        }
       else if (strcmp(cmd, "motor_pos") == 0) {
            char *p1 = strtok(NULL, " "); // 速度 Speed
            char *p2 = strtok(NULL, " "); // 加速度 Accelerate
            char *p3 = strtok(NULL, " "); // 目标位置 Where
            
            if (p1 && p2 && p3) {
                uint16_t speed = (uint16_t)atoi(p1);
                uint8_t accelerate = (uint8_t)atoi(p2);
                uint32_t where = (uint32_t)strtoul(p3, NULL, 10);
                
                // 1. 更新结构体当前时间为系统最新 Tick，以通过盲区时间校验
                my_motor.Current_Time = HAL_GetTick();
                
                // 2. 将定点状态置 1，以通过 Motor_Want_Position 内部的 if(Motor->POINT_State==1) 校验
                my_motor.POINT_State = 1;
                
                // 3. 调用库函数进行位置控制
                uint8_t ret = Motor_Want_Position(&my_motor, speed, accelerate, where);
                
                if (ret == 1) {
                    printf(">> [CLI执行]: 步进电机移动指令已发送。速度: %u, 加速度: %u, 目标位置: %lu\r\n", speed, accelerate, where);
                } else {
                    printf(">> [CLI警告]: 未满足时间间隔(500ms限制)或电机未就绪，控制未执行！\r\n");
                }
            } else {
                printf(">> 参数错误! 格式应为: motor_pos <speed> <accelerate> <where>\r\n");
            }
        }
        else {
           printf(">> 未知指令! 仅支持格式: pos/read/stop/free/lock/motor_pos\r\n");
        }
    }

    
    // 重置缓冲区
    g_cli_rx_index = 0;
    g_cli_frame_ready = false;
}
