/**
 * *****************************************************************************
 * @file    app_servo_fsm.c
 * @brief   舵机状态管理 (Servo Manager FSM) 实现文件
 * @details 负责旋转、对齐和抓爪三个总线舵机的非阻塞控制与到位判断
 * *****************************************************************************
 */

#include "app_servo_fsm.h"
#include <string.h>

/* 全局唯一三个舵机设备实例 */
ServoDevice_t g_servo_base;     /* Base Rotation (ID: 1) */
ServoDevice_t g_servo_align;    /* Grab Alignment (ID: 2) */
ServoDevice_t g_servo_claw;     /* Grab Claw (ID: 3) */

/**
 * @brief  初始化舵机状态与结构体参数
 */
void Servo_App_Init(void)
{
    memset(&g_servo_base, 0, sizeof(ServoDevice_t));
    g_servo_base.id = SERVO_BASE_ROT;
    g_servo_base.state = SERVO_STATE_READY;
    g_servo_base.is_online = true;

    memset(&g_servo_align, 0, sizeof(ServoDevice_t));
    g_servo_align.id = SERVO_GRAB_ALIGN;
    g_servo_align.state = SERVO_STATE_READY;
    g_servo_align.is_online = true;

    memset(&g_servo_claw, 0, sizeof(ServoDevice_t));
    g_servo_claw.id = SERVO_GRAB_CLAW;
    g_servo_claw.state = SERVO_STATE_READY;
    g_servo_claw.is_online = true;
}

/**
 * @brief  舵机的非阻塞状态轮询 (主要进行移动超时检测)
 */
void Servo_App_Update(ServoDevice_t *dev)
{
    if (dev == NULL) return;

    if (dev->state == SERVO_STATE_MOVING)
    {
        /* 如果到位了，切换为 READY */
        if (Servo_App_IsTargetReached(dev))
        {
            dev->state = SERVO_STATE_READY;
        }
        /* 如果运动时间严重超过预期（预期时长 + 1s 超时裕度），则可能堵转或异常 */
        else if (HAL_GetTick() - dev->move_start_tick > dev->expect_duration + 1000)
        {
            dev->state = SERVO_STATE_ERROR;
        }
    }
}

/**
 * @brief  控制指定舵机在规定时间内转到目标位置 (非阻塞)
 */
void Servo_App_SetTarget(ServoDevice_t *dev, int16_t target_pos, uint32_t duration)
{
    if (dev == NULL) return;

    dev->target_pos = target_pos;
    dev->expect_duration = duration;
    dev->move_start_tick = HAL_GetTick();
    dev->state = SERVO_STATE_MOVING;

    /* 调用物理层 API 发送位置控制命令 */
    serial_servo_set_position(&g_serial_servo_controller, dev->id, target_pos, duration);
}

/**
 * @brief  发送非阻塞异步读取指令，将物理层切换回接收，等待中断回调填充
 */
void Servo_App_TriggerRead(ServoDevice_t *dev, uint8_t cmd)
{
    if (dev == NULL) return;

    SerialServoCmdTypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.elements.servo_id = dev->id;
    frame.elements.length = 3;
    frame.elements.command = cmd;
    frame.elements.args[0] = serial_servo_checksum((uint8_t*)&frame);

    /* tx_only = true 发送该读取帧，发送完毕后立即释放总线为接收状态 */
    g_serial_servo_controller.serial_write_and_read(&g_serial_servo_controller, &frame, true);
}

/**
 * @brief  非阻塞判断指定舵机是否运动到位
 */
bool Servo_App_IsTargetReached(ServoDevice_t *dev)
{
    if (dev == NULL) return false;

    /* 正常状态下计算当前回读位置与目标位置的偏差值，15 内属于到位 (0 ~ 1000 范围) */
    int16_t diff = dev->current_pos - dev->target_pos;
    if (diff < 0) diff = -diff;

    if (diff <= 15)
    {
        return true;
    }

    /* 容错：如果移动时间已超过预期耗时，即便回读可能丢包导致没匹配上，也强制判定到位 */
    if (HAL_GetTick() - dev->move_start_tick > dev->expect_duration)
    {
        return true;
    }

    return false;
}

/**
 * @brief  判断是否有任何舵机故障
 */
bool Servo_App_CheckAnyError(void)
{
    /* 如果单设备连续通信失败超过 10 次，或者任何设备状态处于 ERROR 判定为通信异常 */
    if (g_servo_base.err_count > 10 || g_servo_align.err_count > 10 || g_servo_claw.err_count > 10)
    {
        return true;
    }
    if (g_servo_base.state == SERVO_STATE_ERROR || 
        g_servo_align.state == SERVO_STATE_ERROR || 
        g_servo_claw.state == SERVO_STATE_ERROR)
    {
        return true;
    }
    return false;
}

/**
 * @brief  瞬间释放所有舵机扭矩 (掉电)
 */
void Servo_App_UnloadAll(void)
{
    serial_servo_load_unload(&g_serial_servo_controller, SERVO_BASE_ROT, 0);
    serial_servo_load_unload(&g_serial_servo_controller, SERVO_GRAB_ALIGN, 0);
    serial_servo_load_unload(&g_serial_servo_controller, SERVO_GRAB_CLAW, 0);

    g_servo_base.state = SERVO_STATE_READY;
    g_servo_align.state = SERVO_STATE_READY;
    g_servo_claw.state = SERVO_STATE_READY;
}

/**
 * @brief  锁定所有舵机 (上电并保持当前角度)
 */
void Servo_App_LockAll(void)
{
    serial_servo_load_unload(&g_serial_servo_controller, SERVO_BASE_ROT, 1);
    serial_servo_load_unload(&g_serial_servo_controller, SERVO_GRAB_ALIGN, 1);
    serial_servo_load_unload(&g_serial_servo_controller, SERVO_GRAB_CLAW, 1);
}
