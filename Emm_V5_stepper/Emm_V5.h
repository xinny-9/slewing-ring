/**
 * *****************************************************************************
 * @file    Emm_V5.h
 * @brief   Emm_V5.0 闭环步进电机驱动库 (基于 STM32 HAL 库)
 * @details 提取官方例程核心逻辑，提供完整的电机控制、参数修改、回零配置、同步运动，
 *          以及阻塞查询与异步帧解析的双重状态获取机制。
 * *****************************************************************************
 */

#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "main.h"
#include <stdbool.h>

/* 方向定义 */
#define EMM_CW     0   /* 顺时针 */
#define EMM_CCW    1   /* 逆时针 */

/* 阻塞式读取超时时间 (ms) */
#define EMM_BLOCKING_TIMEOUT   100

/* 系统参数类型定义 (用于读取指令) */
typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
} SysParams_t;

/* Emm_V5 步进电机结构体句柄 */
typedef struct {
    uint8_t addr;                  /* 电机通信地址 (1-255) */
    UART_HandleTypeDef *huart;     /* 绑定的 HAL 串口句柄指针 (如 &huart2) */
    
    /* 电机实时遥测数据 */
    uint8_t en_state;              /* 使能状态：0-失能，1-使能 */
    uint8_t arrive_state;          /* 到位状态：0-未到位，1-到位 */
    uint8_t lck_state;             /* 堵转状态：0-正常，1-堵转 */
    uint8_t origin_state;          /* 回零状态：0-未回零/正在回零，1-已完成回零且成功，2-回零失败 */
    
    float real_pos;                /* 电机实时位置角度 (单位: 度) */
    float real_vel;                /* 电机实时转速 (单位: RPM) */
    float err_pos;                 /* 电机实时位置误差角度 (单位: 度) */
    float target_pos;              /* 电机目标位置角度 (单位: 度) */
} Emm_V5_Motor;

/* =============================================================================
 *                              核心控制接口
 * =============================================================================
 */

/**
  * @brief    初始化电机句柄
  * @param    motor : 电机结构体指针
  * @param    huart : 绑定的 HAL 串口句柄
  * @param    addr  : 电机地址 (1-255)
  */
void Emm_V5_Init(Emm_V5_Motor *motor, UART_HandleTypeDef *huart, uint8_t addr);

/**
  * @brief    电机使能控制
  * @param    motor : 电机结构体指针
  * @param    state : 使能状态，true为使能电机，false为关闭电机
  * @param    snF   : 多机同步标志，false为不启用，true为启用
  */
void Emm_V5_En_Control(Emm_V5_Motor *motor, bool state, bool snF);

/**
  * @brief    速度模式控制
  * @param    motor : 电机结构体指针
  * @param    dir   : 方向，0为CW，其余值为CCW
  * @param    vel   : 速度，范围 0 - 5000 RPM
  * @param    acc   : 加速度，范围 0 - 255 (0为直接启动)
  * @param    snF   : 多机同步标志，false为不启用，true为启用
  */
void Emm_V5_Vel_Control(Emm_V5_Motor *motor, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

/**
  * @brief    位置模式控制
  * @param    motor : 电机结构体指针
  * @param    dir   : 方向，0为CW，其余值为CCW
  * @param    vel   : 速度(RPM)，范围 0 - 5000 RPM
  * @param    acc   : 加速度，范围 0 - 255 (0为直接启动)
  * @param    clk   : 脉冲数，范围 0 - (2^32 - 1) 个
  * @param    raF   : 相对/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF   : 多机同步标志，false为不启用，true为启用
  */
void Emm_V5_Pos_Control(Emm_V5_Motor *motor, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);

/**
  * @brief    让电机立即停止运动
  * @param    motor : 电机结构体指针
  * @param    snF   : 多机同步标志，false为不启用，true为启用
  */
void Emm_V5_Stop_Now(Emm_V5_Motor *motor, bool snF);

/**
  * @brief    触发多机同步开始运动
  * @param    motor : 电机结构体指针 (可以是总线上的任意电机地址或广播地址)
  */
void Emm_V5_Synchronous_motion(Emm_V5_Motor *motor);

/**
  * @brief    将当前位置清零 (将当前位置设为零点)
  * @param    motor : 电机结构体指针
  */
void Emm_V5_Reset_CurPos_To_Zero(Emm_V5_Motor *motor);

/**
  * @brief    解除堵转保护
  * @param    motor : 电机结构体指针
  */
void Emm_V5_Reset_Clog_Pro(Emm_V5_Motor *motor);

/**
  * @brief    修改开环/闭环控制模式
  * @param    motor     : 电机结构体指针
  * @param    svF       : 是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode : 控制模式 (1为开环，2为闭环)
  */
void Emm_V5_Modify_Ctrl_Mode(Emm_V5_Motor *motor, bool svF, uint8_t ctrl_mode);

/**
  * @brief    修改电机细分数
  * @param    motor     : 电机结构体指针
  * @param    svF       : 是否存储标志，false为不存储，true为存储
  * @param    subdivide : 细分数值，范围 1 - 255 (例如：16代表16细分，单圈3200脉冲)
  */
void Emm_V5_Modify_Subdivision(Emm_V5_Motor *motor, bool svF, uint8_t subdivide);


/* =============================================================================
 *                              回零控制接口
 * =============================================================================
 */

/**
  * @brief    设置单圈回零的零点位置
  * @param    motor : 电机结构体指针
  * @param    svF   : 是否存储标志，false为不存储，true为存储
  */
void Emm_V5_Origin_Set_O(Emm_V5_Motor *motor, bool svF);

/**
  * @brief    修改回零参数
  * @param    motor  : 电机结构体指针
  * @param    svF    : 是否存储标志，false为不存储，true为存储
  * @param    o_mode : 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  : 回零方向，0为CW，其余值为CCW
  * @param    o_vel  : 回零速度，单位：RPM
  * @param    o_tm   : 回零超时时间，单位：毫秒
  * @param    sl_vel : 无限位碰撞回零检测转速，单位：RPM
  * @param    sl_ma  : 无限位碰撞回零检测电流，单位：mA
  * @param    sl_ms  : 无限位碰撞回零检测时间，单位：毫秒
  * @param    potF   : 上电自动触发回零，false为不使能，true为使能
  */
void Emm_V5_Origin_Modify_Params(Emm_V5_Motor *motor, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

/**
  * @brief    发送命令触发回零
  * @param    motor  : 电机结构体指针
  * @param    o_mode : 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF    : 多机同步标志，false为不启用，true为启用
  */
void Emm_V5_Origin_Trigger_Return(Emm_V5_Motor *motor, uint8_t o_mode, bool snF);

/**
  * @brief    强制中断并退出回零
  * @param    motor : 电机结构体指针
  */
void Emm_V5_Origin_Interrupt(Emm_V5_Motor *motor);


/* =============================================================================
 *                          读取参数与帧解析接口
 * =============================================================================
 */

/**
  * @brief    向电机发送读取系统参数的指令 (非阻塞)
  * @param    motor : 电机结构体指针
  * @param    s     : 系统参数类型
  */
void Emm_V5_Read_Sys_Params(Emm_V5_Motor *motor, SysParams_t s);

/**
  * @brief    阻塞式读取电机当前实时角度 (发送指令并同步等待接收，更新 real_pos)
  * @param    motor : 电机结构体指针
  * @return   bool  : true 读取成功，false 读取失败或超时
  */
bool Emm_V5_Read_Position_Blocking(Emm_V5_Motor *motor);

/**
  * @brief    阻塞式读取电机当前实时转速 (发送指令并同步等待接收，更新 real_vel)
  * @param    motor : 电机结构体指针
  * @return   bool  : true 读取成功，false 读取失败或超时
  */
bool Emm_V5_Read_Speed_Blocking(Emm_V5_Motor *motor);

/**
  * @brief    阻塞式读取电机当前使能/到位/堵转状态 (发送指令并同步等待接收)
  * @param    motor : 电机结构体指针
  * @return   bool  : true 读取成功，false 读取失败或超时
  */
bool Emm_V5_Read_Status_Blocking(Emm_V5_Motor *motor);

/**
  * @brief    统一的帧解析接口 (非阻塞，适用于空闲中断/DMA断帧接收后的数据处理)
  * @param    motor  : 电机结构体指针
  * @param    rx_buf : 接收缓冲区指针
  * @param    rx_len : 接收到的帧长度
  * @return   bool   : true 解析并成功更新数据，false 无效帧或校验失败
  */
bool Emm_V5_Parse_Frame(Emm_V5_Motor *motor, uint8_t *rx_buf, uint8_t rx_len);

#endif /* __EMM_V5_H */
