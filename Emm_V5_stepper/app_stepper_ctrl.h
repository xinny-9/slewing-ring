/**
 * *****************************************************************************
 * @file    app_stepper_ctrl.h
 * @brief   丝杆步进电机应用层控制模块 (基于 Emm_V5 驱动库)
 * @details 封装了物理距离(mm)与脉冲的换算、行程软限位安全保护、上电自动碰撞回零
 *          以及系统状态机，为上层业务提供极其简便、安全的控制接口。
 * *****************************************************************************
 */

#ifndef __APP_STEPPER_CTRL_H
#define __APP_STEPPER_CTRL_H

#include "Emm_V5.h"

/* =============================================================================
 *                                机械结构配置参数
 * =============================================================================
 */
#define SCREW_LEAD_MM         8.0f    /* 丝杆导程：电机旋转一圈，滑块前移 8mm */
#define PULSE_PER_ROUND       3200.0f /* 电机单圈脉冲数 (1.8°步进角，16细分下为 3200 脉冲) */
#define SCREW_MAX_TRAVEL_MM   200.0f  /* 丝杆模组的最大安全物理行程 (单位: mm) */
#define SAFE_CLEARANCE_MM     4.0f    /* 碰撞回零成功后，反向倒车避让的“安全缓冲垫片”距离 */

/* =============================================================================
 *                                系统状态机定义
 * =============================================================================
 */
typedef enum {
    STEPPER_STATE_UNINIT = 0, /* 未初始化 */
    STEPPER_STATE_HOMING,     /* 正在执行碰撞回零 */
    STEPPER_STATE_READY,      /* 回零成功，处于正常绝对位置控制状态 */
    STEPPER_STATE_ERROR       /* 发生通信错误或回零超时 */
} StepperSysState_t;

/* =============================================================================
 *                              上层应用层API接口
 * =============================================================================
 */

/**
  * @brief    应用层丝杆电机系统初始化
  * @param    huart : 电机绑定的 HAL 串口句柄指针 (如 &huart2)
  * @param    addr  : 电机的总线拨码地址 (默认一般为 1)
  */
void Stepper_App_Init(UART_HandleTypeDef *huart, uint8_t addr);

/**
  * @brief    执行上电碰撞自动寻原点流程 (阻塞查询方式)
  * @return   uint8_t : 1-回零成功且零点标定完成；0-回零超时或发生错误
  */
uint8_t Stepper_App_ExecuteHoming(void);

/**
  * @brief    控制滑块移动到丝杆行程的绝对物理位置 (带软限位保护)
  * @param    position_mm : 目标绝对物理位置 (单位: mm，范围 0.0 至 SCREW_MAX_TRAVEL_MM)
  * @param    speed_rpm   : 电机运转的最大速度 (RPM，建议在 500-2000 RPM 之间)
  * @return   uint8_t     : 1-指令合法并已发送；0-参数超出限位保护，被拒绝执行
  */
uint8_t Stepper_App_MoveToPosition(float position_mm, uint16_t speed_rpm);

/**
  * @brief    紧急停止电机的运动
  */
void Stepper_App_EmergencyStop(void);

/**
  * @brief    获取当前电机的软件绝对物理坐标
  * @return   float : 当前位置 (mm)
  */
float Stepper_App_GetCurrentPosition(void);

/**
  * @brief    获取当前丝杆系统所处的运行状态
  * @return   StepperSysState_t : 系统状态机状态
  */
StepperSysState_t Stepper_App_GetSystemState(void);

#endif /* __APP_STEPPER_CTRL_H */
