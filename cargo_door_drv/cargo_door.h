#ifndef __CARGO_DOOR_H
#define __CARGO_DOOR_H

#include "main.h"
#include "tb6612fng.h"
#include <stdio.h>

// 仓门运动动作状态枚举
typedef enum {
    DOOR_STATE_UNINITIALIZED = 0, // 未初始化
    DOOR_STATE_CLOSED,            // 已完全关闭
    DOOR_STATE_OPENING,           // 正在开启
    DOOR_STATE_OPENED,            // 已完全打开
    DOOR_STATE_CLOSING,           // 正在关闭
    DOOR_STATE_STOPPED            // 已手动停止
} DoorState_t;

/* ==============================================
 * ?? 移植与硬件适配可调参数 (移植时只需修改此处)
 * ============================================== */
// 限位开关触发电平 (GPIO_PIN_RESET: 低电平触发(常开接GND); GPIO_PIN_SET: 高电平触发)
#define LIMIT_TRIGGER_LEVEL     GPIO_PIN_RESET 

// 开关启动防抖屏蔽时间 (单位: 毫秒)，防止发车时电机机械抖动误触限位
#define LIMIT_DEBOUNCE_MASK_MS  300

// 默认运行速度 (PWM 占空比, 0 ~ 1000)
#define MOTOR_DEFAULT_SPEED     800
/* ============================================== */

// 仓门控制结构体封装
typedef struct {
    const char*   Name;           // 仓门名称标识 (如 "Door_Top")
    TB6612_MotorTypeDef* Motor;   // 关联的电机控制结构体
    
    // 开仓、关仓两个接近开关接口绑定
    GPIO_TypeDef* Limit_Open_Port;
    uint16_t      Limit_Open_Pin;
    GPIO_TypeDef* Limit_Close_Port;
    uint16_t      Limit_Close_Pin;

    DoorState_t   State;          // 运行状态
    uint32_t      timer_start;    // 状态切换时间戳
} CargoDoor_t;

// 声明全局左仓门、右仓门句柄
extern CargoDoor_t door_top;
extern CargoDoor_t door_lower;

/* 驱动库核心 API */
void CargoDoor_Init(CargoDoor_t* door, const char* name, TB6612_MotorTypeDef* motor,
                    GPIO_TypeDef* open_port, uint16_t open_pin,
                    GPIO_TypeDef* close_port, uint16_t close_pin);

void CargoDoor_Update(CargoDoor_t* door);
void CargoDoor_Open(CargoDoor_t* door);
void CargoDoor_Close(CargoDoor_t* door);
void CargoDoor_Stop(CargoDoor_t* door);
void CargoDoor_PrintStatus(CargoDoor_t* door);

#endif /* __CARGO_DOOR_H */
