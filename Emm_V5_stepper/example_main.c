/**

 * *****************************************************************************

 * @file    example_main.c

 * @brief   Emm_V5.0 闭环步进电机驱动库使用例程 (基于 STM32 HAL 库)

 * @details 演示如何初始化电机、使能、控制速度/位置、进行阻塞式参数读取，

 *          以及如何在 HAL 的串口接收中断中以非阻塞异步方式解算电机遥测数据。

 * *****************************************************************************

 */



#include "main.h"

#include "usart.h"

#include "Emm_V5.h"

#include "app_stepper_ctrl.h"

#include <stdio.h>



/* 声明步进电机句柄 */

Emm_V5_Motor stepper;



/* 串口异步接收相关的缓存定义 (假设使用串口2连接电机) */

#define RX_BUFFER_SIZE  64

uint8_t g_stepper_rx_buf[RX_BUFFER_SIZE];

uint8_t g_stepper_rx_len = 0;



/**

  * @brief  电机演示主业务逻辑

  */

void Stepper_Demo_Process(void)

{

    /* 1. 初始化电机句柄：绑定串口2 (&huart2)，设置电机总线地址为1 */

    Emm_V5_Init(&stepper, &huart2, 1);

    

    /* 2. 确保电机驱动器上电稳定后，使能电机 */

    HAL_Delay(1000); 

    Emm_V5_En_Control(&stepper, true, false);

    HAL_Delay(200);



    /* 3. 阻塞式读取当前电机的状态，验证通信是否正常 */

    printf(">> 正在阻塞式读取电机初始状态...\r\n");

    if (Emm_V5_Read_Status_Blocking(&stepper))

    {

        printf(">> 读取成功！当前使能状态：%d, 到位状态：%d, 堵转状态：%d\r\n", 

               stepper.en_state, stepper.arrive_state, stepper.lck_state);

    }

    else

    {

        printf(">> 通信失败！请检查电机接线与地址设置。\r\n");

    }



    /* 4. 修改回零参数，并触发无限位碰撞回零动作 */

    printf(">> 正在配置碰撞回零参数...\r\n");

    /* 参数: 句柄, 存储, 模式2(碰撞回零), 方向CCW, 回零速50RPM, 超时10s, 碰撞检测速10RPM, 电流300mA, 时间100ms, 上电不自动触发 */

    Emm_V5_Origin_Modify_Params(&stepper, true, HOMING_MODE, HOMING_DIR, HOMING_SPEED_RPM, HOMING_TIMEOUT_MS, HOMING_SL_VEL_RPM, HOMING_SL_CUR_MA, HOMING_SL_TIME_MS, HOMING_AUTO_START);

    HAL_Delay(100);

    

    printf(">> 触发回零动作...\r\n");

    stepper.origin_state = 0xFF;

    Emm_V5_Origin_Trigger_Return(&stepper, 2, false);

    

    /* 阻塞等待回零成功 (也可以结合中断非阻塞检测 origin_state) */

    uint32_t timeout_tick = HAL_GetTick();

    while (1)

    {

        /* 间隔500ms查询一次回零状态 */

        HAL_Delay(500);

        Emm_V5_Read_Sys_Params(&stepper, S_ORG); // 非阻塞发送查询指令，等待中断回调中解析

        

        /* 检查解析出来的状态 (1-成功，2-失败) */

        if (stepper.origin_state != 0xFF && (stepper.origin_state & 0x04) == 0)

        {

            printf(">> 回零成功！当前位置已被设为零点。\r\n");

            break;

        }

        else if (stepper.origin_state == 0)

        {

            printf(">> 回零失败！\r\n");

            break;

        }

        

        /* 超时退出保护 (15秒) */

        if (HAL_GetTick() - timeout_tick > 15000)

        {

            printf(">> 回零检测超时！\r\n");

            Emm_V5_Origin_Interrupt(&stepper); // 强制中断回零

            break;

        }

    }



    /* 5. 位置控制演示：相对运动 */

    printf(">> 运动演示：顺时针旋转一圈 (在16细分下发送3200个脉冲，速度1000RPM，加速度5)\r\n");

    Emm_V5_Pos_Control(&stepper, EMM_CW, 1000, 5, 3200, false, false);

    

    /* 轮询等待到位标志被置位 */

    while (1)

    {

        HAL_Delay(200);

        Emm_V5_Read_Status_Blocking(&stepper);

        if (stepper.arrive_state == 1)

        {

            printf(">> 目标位置到达！\r\n");

            break;

        }

    }



    /* 6. 阻塞式获取当前实时角度与转速 */

    if (Emm_V5_Read_Position_Blocking(&stepper))

    {

        printf(">> 电机当前位置角度为: %.2f 度\r\n", stepper.real_pos);

    }

    

    /* 7. 速度模式控制：以 500 RPM 持续运转，并在 3 秒后停止 */

    printf(">> 速度模式：CCW方向以500RPM运转...\r\n");

    Emm_V5_Vel_Control(&stepper, EMM_CCW, 500, 10, false);

    

    HAL_Delay(3000);

    

    printf(">> 立即停转！\r\n");

    Emm_V5_Stop_Now(&stepper, false);

}





/**

  * @brief  标准 HAL 库串口接收中断完成回调函数 (USER CODE 4 区域)

  * @note   在此处对称、零侵入地分发处理来自不同串口设备的数据包

  */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{

    /* 处理来自电机串口 (huart2) 的回传字节数据 */

    if (huart->Instance == huart2.Instance)

    {

        /*

         * 示例一：如果您使用“单字节中断接收”配合队列/状态机断帧：

         * 接收到一个字节（如 g_servo_temp_byte）后将其推入缓冲区：

         * g_stepper_rx_buf[g_stepper_rx_len++] = g_servo_temp_byte;

         * 

         * 并在判断一帧接收完毕时（例如通过时间间隔、协议长度、或空闲中断）：

         * Emm_V5_Parse_Frame(&stepper, g_stepper_rx_buf, g_stepper_rx_len);

         * g_stepper_rx_len = 0; // 重置计数

         */

        

        /* 

         * 示例二：如果您使用“串口空闲中断 (IDLE) + DMA接收”：

         * 在空闲中断 ISR 中，计算出当前接收到的字节数 rx_len，然后直接执行：

         * Emm_V5_Parse_Frame(&stepper, dma_rx_buffer, rx_len);

         */

    }

}

