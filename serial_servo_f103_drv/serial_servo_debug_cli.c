/**
 * *****************************************************************************
 * @file    serial_servo_debug_cli.c
 * @brief   控制台指令捕获与解析状态机映射源文件 (USART3)
 * *****************************************************************************
 */

#include "serial_servo_debug_cli.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../crane_system_fsm/app_system_fsm.h"

#define CLI_RX_LEN 64
static uint8_t g_cli_rx_buf[CLI_RX_LEN];
static uint8_t g_cli_rx_index = 0;
static volatile bool g_cli_frame_ready = false;
/* 串口 3 异步接收单字节接收缓冲区 */
static uint8_t g_cli_rx_temp_byte = 0;

/**
 * @brief  初始化调试命令行控制台并开启调试串口 3 接收中断
 */
void Debug_CLI_Init(void)
{
    g_cli_rx_index = 0;
    g_cli_frame_ready = false;
    
    // 强制清除 ORE 挂起标志，防止上电产生溢出死锁
    __HAL_UART_CLEAR_OREFLAG(&DEBUG_CLI_UART);
    
    // 开启调试串口 3 异步 1 字节接收中断
    HAL_UART_Receive_IT(&DEBUG_CLI_UART, &g_cli_rx_temp_byte, 1);
    
    // 向上位机串口助手输出操作菜单
    printf("\r\n==================================================\r\n");
    printf("   [*] 串口总线舵机 STM32F103 调试 CLI 控制台就绪 (USART3)\r\n");
    printf("   使用说明: 在串口助手输入以下指令 (请勾选发送新行):\r\n");
    printf("     1. pos <id> <pos> <time>  -> 控制舵机运动到目标位置 (0~1000)\r\n");
    printf("     2. read <id>             -> 回读舵机位置、电压和温度\r\n");
    printf("     3. stop <id>             -> 紧急停止锁死\r\n");
    printf("     4. free <id>             -> 释放力矩 (手动教学示教)\r\n");
    printf("     5. lock <id>             -> 重新上电力矩锁定\r\n");
    printf("     6. motor_pos <pos> <speed> -> 步进电机位置控制(Emm_V5)\r\n");
    printf("     7. seq                   -> 启动全自动抓取工艺流程\r\n");
    printf("     8. next                  -> [快捷单步] 顺序触发并运行下一步\r\n");
    printf("     9. step <1~10>           -> 触发执行 1~10 步指定单步调试\r\n");
    printf("     10. set_align <pos>      -> 动态对齐货箱角度 (0~1000)\r\n");
    printf("     11. status               -> 打印系统全部设备状态遥测\r\n");
    printf("     12. mode <auto/manual>   -> 动态切换系统运行模式 (默认: auto)\r\n");
    printf("==================================================\r\n\r\n");
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

    // 捕获回车或换行作为帧结束符
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
 * @brief  主循环调用命令解析与起重机动作映射处理
 */
void Debug_CLI_Process(void)
{
    if (!g_cli_frame_ready) {
        return;
    }
    
    // 使用 strtok 切割传入的 ASCII 命令行参数
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
                // 直接控制总线舵机运动
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
                
                printf(">> [CLI执行]: 正在从总线回读舵机 %d 实时参数...\r\n", id);
                
                // 顺序执行回读，每步动作之间保留 40ms 物理通道静默间歇，以保障抗干扰
                int r1 = serial_servo_read_position(&g_serial_servo_controller, id, &pos);
                HAL_Delay(40);
                int r2 = serial_servo_read_vin(&g_serial_servo_controller, id, &vin);
                HAL_Delay(40);
                int r3 = serial_servo_read_temp(&g_serial_servo_controller, id, &temp);
                
                if (r1 == 0 && r2 == 0 && r3 == 0) {
                    printf(">> [回读成功] 实时位置: %4d | 供电电压: %.2f V | 芯片温度: %d ℃\r\n", pos, vin/1000.0f, temp);
                } else {
                    printf(">> [回读失败] 舵机无响应！请排查供电或ID配置。(r_pos:%d, r_vin:%d, r_temp:%d)\r\n", r1, r2, r3);
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
                printf(">> [CLI执行]: 紧急制动舵机 %d\r\n", id);
            } else {
                printf(">> 参数错误! 格式应为: stop <id>\r\n");
            }
        }
        else if (strcmp(cmd, "free") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                int id = atoi(p1);
                serial_servo_load_unload(&g_serial_servo_controller, id, 0);
                printf(">> [CLI执行]: 释放舵机 %d 力矩 (进入手动示教模式)\r\n", id);
            } else {
                printf(">> 参数错误! 格式应为: free <id>\r\n");
            }
        }
        else if (strcmp(cmd, "lock") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                int id = atoi(p1);
                serial_servo_load_unload(&g_serial_servo_controller, id, 1);
                printf(">> [CLI执行]: 锁定舵机 %d 力矩\r\n", id);
            } else {
                printf(">> 参数错误! 格式应为: lock <id>\r\n");
            }
        }
        else if (strcmp(cmd, "motor_pos") == 0) {
            char *p1 = strtok(NULL, " ");
            char *p2 = strtok(NULL, " ");
            if (p1 && p2) {
                float pos_mm = atof(p1);
                int speed_rpm = atoi(p2);
                if (Stepper_App_MoveToPosition(pos_mm, speed_rpm) == 1) {
                    printf(">> [CLI执行]: Emm_V5 电机向目标位置 %.1f mm 移动，速度 %d RPM\r\n", pos_mm, speed_rpm);
                } else {
                    printf(">> [CLI执行]: 移动指令被拒绝！电机处于未就绪/回零/故障状态\r\n");
                }
            } else {
                printf(">> 参数错误! 格式应为: motor_pos <pos> <speed>\r\n");
            }
        }
        else if (strcmp(cmd, "seq") == 0) {
            if (System_FSM_StartSequence() == 1) {
                printf(">> [CLI执行]: 起重机状态机全自动抓取工艺序列启动成功...\r\n");
            } else {
                printf(">> [CLI执行]: 启动拒绝！系统当前不处于 READY 状态\r\n");
            }
        }
        else if (strcmp(cmd, "next") == 0) {
            uint8_t triggered_step = System_FSM_StartNextSingleStep();
            if (triggered_step > 0) {
                printf(">> [CLI执行]: [顺次单步] 成功触发起重机第 %d 步动作...\r\n", triggered_step);
            } else {
                printf(">> [CLI执行]: 触发失败！状态机未就绪或当前动作尚未结束\r\n");
            }
        }
        else if (strcmp(cmd, "step") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                int step_num = atoi(p1);
                if (System_FSM_StartSingleStep(step_num) == 1) {
                    printf(">> [CLI执行]: 成功触发运行指定单步动作 %d...\r\n", step_num);
                } else {
                    printf(">> [CLI执行]: 触发拒绝！步骤必须在 1~10 之间或状态机未就绪\r\n");
                }
            } else {
                printf(">> 参数错误! 格式应为: step <1~10>\r\n");
            }
        }
        else if (strcmp(cmd, "set_align") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                int align_pos = atoi(p1);
                System_FSM_SetGrabAlignPos((uint16_t)align_pos);
            } else {
                printf(">> 参数错误! 格式应为: set_align <pos>\r\n");
            }
        }
        else if (strcmp(cmd, "status") == 0) {
            char telemetry_buf[256];
            System_FSM_GetStatusString(telemetry_buf, sizeof(telemetry_buf));
            printf("%s\r\n", telemetry_buf);
        }
        else if (strcmp(cmd, "mode") == 0) {
            char *p1 = strtok(NULL, " ");
            if (p1) {
                if (strcmp(p1, "auto") == 0) {
                    System_FSM_SetControlMode(SYS_MODE_AUTO);
                } else if (strcmp(p1, "manual") == 0) {
                    System_FSM_SetControlMode(SYS_MODE_MANUAL);
                } else {
                    printf(">> 参数错误! 格式应为: mode <auto/manual>\r\n");
                }
            } else {
                printf(">> 参数错误! 格式应为: mode <auto/manual>\r\n");
            }
        }
        else {
           printf(">> 未知指令! 仅支持格式: pos/read/stop/free/lock/motor_pos/seq/next/step/set_align/status/mode\r\n");
        }
    }
    
    // 重置缓冲区
    g_cli_rx_index = 0;
    g_cli_frame_ready = false;
}
