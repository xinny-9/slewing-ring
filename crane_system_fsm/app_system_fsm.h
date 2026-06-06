/**
 * *****************************************************************************
 * @file    app_system_fsm.h
 * @brief   起重器系统级主控制状态机 (Master FSM) 头文件
 * @details 协调升降步进电机和三个总线舵机以非阻塞方式联动，
 *          包含所有可配置的物理高度、角度、延时和判定容差宏定义。
 * *****************************************************************************
 */

#ifndef __APP_SYSTEM_FSM_H
#define __APP_SYSTEM_FSM_H

#include "stm32f1xx_hal.h"
#include "app_servo_fsm.h"
#include "../Emm_V5_stepper/app_stepper_ctrl.h"

/* =============================================================================
 *                               调试参数宏定义清单
 * =============================================================================
 */

/* A. 升降高度位置参数 (步进电机 - 精度: 0.01mm) */
#define ELEV_HEIGHT_SAFE            (20.0f)     /* 搬运与旋转过程中的安全悬挂高度 (mm)，防拖地碰壁 */
#define ELEV_HEIGHT_GRAB            (150.0f)    /* 抓取货物时的下降高度 (mm) */
#define ELEV_HEIGHT_DROP            (100.0f)    /* 释放货物时的安全下降高度 (mm) */
#define ELEV_HEIGHT_MAX_LIMIT       (3500.0f)   /* 升降机构最大安全物理行程软限位 (mm) */

/* B. 水平角度及对齐参数 (总线舵机 1 & 2 - 范围: 0 ~ 1000) */
#define BASE_ROT_POS_START          (100)       /* 初始对准货物的底盘角度 (舵机1) */
#define BASE_ROT_POS_BOX            (600)       /* 货箱正上方的底盘旋转角度 (舵机1) */
#define BASE_ROT_MIN_LIMIT          (50)        /* 底座旋转安全最小限位 */
#define BASE_ROT_MAX_LIMIT          (950)       /* 底座旋转安全最大限位 */
#define GRAB_ALIGN_POS_START        (100)       /* 抓取时抓斗的初始对齐朝向 (舵机2) */
#define GRAB_ALIGN_POS_BOX          (400)       /* 货箱方向抓斗对准角度缺省值 (仅作为上电初值) */

/* C. 爪子完全开合位置 (总线舵机 3 - 范围: 0 ~ 1000) */
#define GRAB_CLAW_POS_OPEN          (200)       /* 爪子完全张开位置脉冲值 */
#define GRAB_CLAW_POS_CLOSE         (750)       /* 爪子完全闭合夹紧位置脉冲值 */

/* D. "顿戳微张式二次深挖" 专有工艺参数 */
#define ELEV_FIRST_DIG_DEPTH        (6.0f)      /* 第一次伴随浅压挖掘深度 (mm) */
#define ELEV_RETRACT_HEIGHT         (8.0f)      /* 第一次压完后，向上抬起释放硬应力的距离 (mm) */
#define ELEV_SECOND_DIG_DEPTH       (15.0f)     /* 第二次全力深入挖掘压入的绝对深度 (mm) */
#define CLAW_MID_CLOSE_POS          (450)       /* 第一次下压时，爪子半闭合聚拢角度 (脉冲) */
#define CLAW_MID_BACK_POS           (350)       /* 抬起释放应力时，爪子微幅向外退回张开的角度 (脉冲) */

/* E. 时间与速度配置参数 (S曲线及运动时间) */
#define STEPPER_SPEED_ELEV          (800)       /* 升降步进电机的移动速度 (RPM) */
#define STEPPER_ACC_ELEV            (15)        /* 升降电机加减速档位 (0 ~ 15，S曲线平滑防抖) */
#define BASE_ROT_DURATION_MS        (1800)      /* 底座大范围水平旋转时间 (ms)，缓慢旋转防晃 */
#define GRAB_ALIGN_DURATION_MS      (800)       /* 抓斗对齐旋转所用时间 (ms) */
#define GRAB_CLAW_DURATION_MS       (600)       /* 爪子张合运行所用时间 (ms) */
#define DELAY_GRAB_SETTLE_MS        (1000)      /* 爪子完全咬紧后，起吊前的物理稳定延时 (ms) */
#define DELAY_DROP_SETTLE_MS        (800)       /* 爪子完全张开后，物料脱开落稳的等待延时 (ms) */

/* F. 到位判定容差 */
#define TOLERANCE_STEPPER_MM        (1.5f)      /* 步进电机到位判定绝对差值容差 (mm) */

/* =============================================================================
 *                               系统级状态枚举
 * =============================================================================
 */

/* 全局主状态 */
typedef enum {
    SYS_STATE_UNINIT = 0,               /* 系统上电未初始化 */
    SYS_STATE_HOMING_STEPPER,           /* 升降电机正在执行碰撞回零 */
    SYS_STATE_DETECT_SERVOS,            /* 升降就绪后，探测扫描三个舵机在线状态 */
    SYS_STATE_READY,                    /* 系统就绪待命状态 (低频遥测监控模式) */
    SYS_STATE_RUNNING_SEQUENCE,         /* 正在执行起重搬运联合联动动作序列 */
    SYS_STATE_WAIT_SERVO_REPLY,         /* 非阻塞等待舵机应答的公共过渡状态 */
    SYS_STATE_ERROR                     /* 全局故障紧急断动力矩保护状态 */
} SystemState_t;

/* 联动工步子状态 */
typedef enum {
    SYS_TASK_IDLE = 0,
    SYS_TASK_STEP_1_RAISE_SAFE,         /* 步骤1: 升降先缩回至安全提升高度 */
    SYS_TASK_STEP_2_OPEN_CLAW,          /* 步骤2: 爪子完全张开，抓斗旋转对准初始朝向 */
    SYS_TASK_STEP_3_DESCEND_GRAB,       /* 步骤3: 升降下降至预备抓取高度 */
    SYS_TASK_STEP_4_1_FIRST_DIG,        /* 步骤4-1: 第一次半咬合并同步微下压 */
    SYS_TASK_STEP_4_1_WAIT,             /* 步骤4-1 等待到位 */
    SYS_TASK_STEP_4_2_RETRACT_SETTLE,   /* 步骤4-2: 向上抬起一小段，且爪子微张 (消除结拱硬阻力) */
    SYS_TASK_STEP_4_2_WAIT,             /* 步骤4-2 等待到位 */
    SYS_TASK_STEP_4_2_DELAY,            /* 步骤4-2 等待散装颗粒物料完全流动坍塌 */
    SYS_TASK_STEP_4_3_SECOND_DIG_LOCK,  /* 步骤4-3: 向下深入最大挖掘深度，同时爪子全力闭合咬紧 */
    SYS_TASK_STEP_4_3_WAIT,             /* 步骤4-3 等待到位 */
    SYS_TASK_STEP_4_3_SETTLE,           /* 步骤4-3 抓紧后的抓取物料稳定延时 */
    SYS_TASK_STEP_5_RAISE_SAFE,         /* 步骤5: 升降起吊至安全搬运高度 */
    SYS_TASK_STEP_6_ROTATE_TO_BOX,      /* 步骤6: 底座水平旋转至货箱正上方，同时抓斗旋转对准货箱 */
    SYS_TASK_STEP_7_DESCEND_DROP,       /* 步骤7: 升降下降至卸载安全释放高度 */
    SYS_TASK_STEP_8_RELEASE_CLAW,       /* 步骤8: 爪子张开释放货物 */
    SYS_TASK_STEP_8_WAIT_SETTLE,        /* 步骤8 释放后的物料落稳延时 */
    SYS_TASK_STEP_9_RAISE_AFTER_RELEASE,/* 步骤9: 起吊回安全高度 */
    SYS_TASK_STEP_10_RETURN_START,      /* 步骤10: 底座及对准回转复位至初始抓取点 */
    SYS_TASK_DONE                       /* 搬运流程顺利结束 */
} SystemTaskStep_t;

/* =============================================================================
 *                              外部 API 接口声明
 * =============================================================================
 */

/**
 * @brief  初始化起重器系统级联合状态机
 */
void System_FSM_Init(void);

/**
 * @brief  系统状态机核心调度 Process (应放置在 main 的 while(1) 中以 50ms 节拍被轮询)
 */
void System_FSM_Process(void);

/**
 * @brief  一键触发执行全局起重搬运联合联动动作流程 (seq 指令映射)
 * @retval 1-启动成功；0-当前系统未就绪，拒绝执行
 */
uint8_t System_FSM_StartSequence(void);

/**
 * @brief  执行单步工步调试流转 (step 指令映射)
 * @param  step_num: 指定执行的单步序号 (1 ~ 10)
 * @retval 1-启动成功；0-当前系统未就绪或参数错误，拒绝执行
 */
uint8_t System_FSM_StartSingleStep(uint8_t step_num);

/**
 * @brief  全局安全紧急停车 (制动升降电机并彻底切断舵机力矩)
 */
void System_FSM_EmergencyStop(void);

/**
 * @brief  供上位机调用：动态微调更新对准货箱时抓斗的对齐目标值
 * @param  pos: 动态目标角度值 (0 ~ 1000)
 */
void System_FSM_SetGrabAlignPos(uint16_t pos);

/**
 * @brief  获取并生成全系统状态与各机构的运行数据，供控制台 status 命令回显
 * @param  buf: 存储状态字符的缓冲区指针
 * @param  len: 缓冲区最大可用长度
 */
/* =============================================================================
 *                            运行模式定义
 * =============================================================================
 */
typedef enum {
    SYS_MODE_MANUAL = 0, /* 手动/单步模式 */
    SYS_MODE_AUTO        /* 自动模式 */
} SystemControlMode_t;

#define DEFAULT_SYS_MODE      SYS_MODE_AUTO /* 默认配置为自动模式 */

void System_FSM_GetStatusString(char *buf, uint16_t len);

/**
  * @brief  顺序触发执行下一步单步工步 (next 指令触发)
  * @retval 触发成功的工步编号 (1~10)，0 表示触发失败
  */
uint8_t System_FSM_StartNextSingleStep(void);

/**
  * @brief  设置起重机控制系统工作模式 (mode 指令触发)
  * @param  mode: 运行模式 (SYS_MODE_MANUAL / SYS_MODE_AUTO)
  */
void System_FSM_SetControlMode(SystemControlMode_t mode);

/* =============================================================================
 *                            时钟中断全局变量
 * =============================================================================
 */
extern volatile uint8_t  g_fsm_update_flag;                 /* 50ms 状态查询执行标志 */
extern volatile uint8_t  g_single_step_only;                /* 单步模式标志：1-单步；0-自动 */
extern volatile uint32_t g_servo_reply_timeout_counter;     /* 舵机 10ms 延时计时器 */
extern volatile uint32_t g_sequence_settle_counter;         /* 动作稳定时间计时器 */
extern volatile uint32_t g_settle_delay_counter;           /* 延时稳定时间计时器 */

/* 货箱对准的偏转角度 */
extern volatile uint16_t g_grab_align_pos_box;

/* 全局工作模式变量 */
extern volatile SystemControlMode_t g_system_control_mode;

#endif /* __APP_SYSTEM_FSM_H */
