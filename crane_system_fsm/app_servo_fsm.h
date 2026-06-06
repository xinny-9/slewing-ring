/**
 * *****************************************************************************
 * @file    app_servo_fsm.h
 * @brief   起重器总线舵机独立状态机控制头文件
 * @details 负责管理底座、抓斗对准、爪子开合三个总线舵机独立的生命周期状态机，
 *          提供非阻塞的高/低频轮询更新与异步回读应答接口。
 * *****************************************************************************
 */

#ifndef __APP_SERVO_FSM_H
#define __APP_SERVO_FSM_H

#include "../serial_servo_f103_drv/serial_servo.h"
#include "../serial_servo_f103_drv/serial_servo_hal.h"

/* 拼接高低字节宏定义 (解决跨源文件符号引用未定义报错) */
#ifndef BYTE_TO_HW
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
#endif
#include "stm32f1xx_hal.h"

/* =============================================================================
 *                                任务舵机 ID 定义
 * =============================================================================
 */
#define SERVO_BASE_ROT           (6)   /* 舵机1：控制底座水平旋转角度 (Base Rotation) */
#define SERVO_GRAB_ALIGN         (9)   /* 舵机2：控制抓斗水平旋转对准方向 (Grab Alignment) */
#define SERVO_GRAB_CLAW          (1)   /* 舵机3：控制抓爪完全开合/抓放 (Grab Claw Open/Close) */

/* =============================================================================
 *                                舵机状态枚举定义
 * =============================================================================
 */
typedef enum {
    SERVO_STATE_UNINIT = 0,            /* 未初始化状态 */
    SERVO_STATE_DETECTING,             /* 正在发送在线探测命令 */
    SERVO_STATE_READY,                 /* 就绪/空闲状态 (低频参数轮询模式) */
    SERVO_STATE_MOVING,                /* 运动中状态 (高频位置轮询以判断到位) */
    SERVO_STATE_ERROR                  /* 通信丢失或异常过载故障状态 */
} ServoState_t;

/* =============================================================================
 *                             单个总线舵机设备结构体
 * =============================================================================
 */
typedef struct {
    uint8_t id;                        /* 舵机物理 ID (1, 2, 3) */
    ServoState_t state;                /* 舵机当前状态机的状态 */
    int16_t current_pos;               /* 回读的实时角度/位置值 (0 ~ 1000) */
    int16_t target_pos;                /* 设定的目标运行位置 (0 ~ 1000) */
    uint32_t move_start_tick;          /* 开始执行运动时的时间戳 (ms) */
    uint32_t expect_duration;          /* 预期的运行总时间 (ms) */
    uint8_t temp;                      /* 实时回读的芯片温度 (℃) */
    uint16_t vin;                      /* 实时回读的输入电压 (mV) */
    uint32_t err_count;                /* 连续通信失败/超时的次数计数 */
    bool is_online;                    /* 舵机在线标志 */
} ServoDevice_t;

/* =============================================================================
 *                              外部可调用 API 接口
 * =============================================================================
 */

/**
 * @brief  初始化三个任务舵机的结构体参数与状态机
 */
void Servo_App_Init(void);

/**
 * @brief  非阻塞式更新单个舵机的状态机 (需以 50ms 节拍在系统 FSM 中轮询调用)
 * @param  dev: 指向对应舵机设备结构体的指针
 */
void Servo_App_Update(ServoDevice_t *dev);

/**
 * @brief  控制指定舵机在规定时间内转动到目标位置 (非阻塞发起命令)
 * @param  dev: 目标舵机设备指针
 * @param  target_pos: 目标位置 (0 ~ 1000)
 * @param  duration: 运动所用时间 (ms)
 */
void Servo_App_SetTarget(ServoDevice_t *dev, int16_t target_pos, uint32_t duration);

/**
 * @brief  向指定的舵机发起参数异步回读请求 (只管发送，不原地 while 死等)
 * @param  dev: 目标舵机设备指针
 * @param  cmd: 读指令类型 (如 SERIAL_SERVO_POS_READ / SERIAL_SERVO_VIN_READ 等)
 */
void Servo_App_TriggerRead(ServoDevice_t *dev, uint8_t cmd);

/**
 * @brief  判断指定舵机是否已经运动到达了目标容差范围
 * @param  dev: 目标舵机设备指针
 * @retval true-已到达目标位置或超时强制退出；false-仍在运动中
 */
bool Servo_App_IsTargetReached(ServoDevice_t *dev);

/**
 * @brief  检查是否有任何一个舵机发生了严重的过热或硬件故障
 * @retval true-存在硬件故障；false-一切正常
 */
bool Servo_App_CheckAnyError(void);

/**
 * @brief  紧急卸载三个总线舵机的所有扭矩力矩 (使能释放，防卡死烧毁)
 */
void Servo_App_UnloadAll(void);

/**
 * @brief  重新加锁/使能三个总线舵机的力矩锁定
 */
void Servo_App_LockAll(void);

/* 导出全局唯一的三个舵机设备对象实例 */
extern ServoDevice_t g_servo_base;     /* 底座旋转舵机 */
extern ServoDevice_t g_servo_align;    /* 抓斗对齐舵机 */
extern ServoDevice_t g_servo_claw;     /* 爪子开合舵机 */

#endif /* __APP_SERVO_FSM_H */
