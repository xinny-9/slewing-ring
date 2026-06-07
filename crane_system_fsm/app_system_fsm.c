/**
 * *****************************************************************************
 * @file    app_system_fsm.c
 * @brief   起重器系统级主控制状态机 (Master FSM) 源文件
 * *****************************************************************************
 */

#include "app_system_fsm.h"
#include <stdio.h>
#include <string.h>

/* 定时器中断相关的全局标志与递减计数器声明 */
volatile uint8_t  g_fsm_update_flag = 0;
volatile uint8_t  g_single_step_only = 0;
volatile uint32_t g_servo_reply_timeout_counter = 0;
volatile uint32_t g_sequence_settle_counter = 0;
volatile uint32_t g_settle_delay_counter = 0;

/* 上位机可修改的抓斗放箱对齐角度全局变量 */
volatile uint16_t g_grab_align_pos_box = GRAB_ALIGN_POS_BOX;
volatile uint16_t g_grab_align_pos_start = 100; /* 抓取时抓斗的初始对齐朝向 (舵机2) */

/* 系统状态机当前状态与搬运序列工步 */
static SystemState_t g_system_state = SYS_STATE_UNINIT;
static SystemTaskStep_t g_seq_step = SYS_TASK_IDLE;

/* 暂存等待响应前的状态与目标舵机设备指针 */
static SystemState_t g_saved_pre_state = SYS_STATE_UNINIT;
static ServoDevice_t *g_waiting_dev_ptr = NULL;
static uint8_t g_next_single_step_num = 1; /* 顺序单步调试下一步指示器 */
volatile SystemControlMode_t g_system_control_mode = DEFAULT_SYS_MODE; /* 全局工作模式 */

/* 内部辅助函数：获取当前工步的目标舵机实例指针 */
static ServoDevice_t* get_servo_device_by_id(uint8_t id)
{
    if (id == SERVO_BASE_ROT)   return &g_servo_base;
    if (id == SERVO_GRAB_ALIGN) return &g_servo_align;
    if (id == SERVO_GRAB_CLAW)  return &g_servo_claw;
    return NULL;
}

/**
 * @brief  初始化起重器系统级联合状态机
 */
void System_FSM_Init(void)
{
    g_system_state = SYS_STATE_UNINIT;
    g_seq_step = SYS_TASK_IDLE;
    g_grab_align_pos_box = GRAB_ALIGN_POS_BOX;
    g_grab_align_pos_start = 100;
    
    g_fsm_update_flag = 0;
    g_single_step_only = 0;
    g_servo_reply_timeout_counter = 0;
    g_sequence_settle_counter = 0;
    g_settle_delay_counter = 0;
    
    g_waiting_dev_ptr = NULL;
    g_next_single_step_num = 1;
    g_system_control_mode = DEFAULT_SYS_MODE;
    
    /* 调用并初始化子层舵机结构体 */
    Servo_App_Init();
}

/**
 * @brief  向指定的舵机发起位置异步读取请求，并暂时切换状态机以进行非阻塞等待
 */
static void trigger_servo_read_blocking_alternative(ServoDevice_t *dev, uint8_t cmd, SystemState_t current_state)
{
    g_waiting_dev_ptr = dev;
    g_saved_pre_state = current_state;
    
    /* 1. 发起非阻塞读取请求 (只管发送，不在原地 while 等待) */
    Servo_App_TriggerRead(dev, cmd);
    
    /* 2. 设置 10ms 通信等待超时计时器 (50ms 周期内 10ms 足够响应) */
    g_servo_reply_timeout_counter = 3; 
    
    /* 3. 切换状态至非阻塞等待响应 */
    g_system_state = SYS_STATE_WAIT_SERVO_REPLY;
}

/**
 * @brief  过渡跳转辅助函数（在单步调试模式下起到阻断自动跳转的作用）
 * @param  next_step: 连续模式下的下一步工步
 * @param  current_step_num: 当前工步的数字序号 (1 ~ 10)
 */
static void transition_to_next_step(SystemTaskStep_t next_step, uint8_t current_step_num)
{
    /* 如果是调试单步指令触发，或是处于手动模式，到位后一律安全暂停拦截 */
    if (g_single_step_only || g_system_control_mode == SYS_MODE_MANUAL)
    {
        g_seq_step = SYS_TASK_IDLE;
        g_system_state = SYS_STATE_READY;
        printf(">> [系统暂停]: 第 %d 步动作到位，系统已安全暂停并回归 READY。\r\n", current_step_num);
    }
    else
    {
    /* 自动模式下，自动流转到下一步 */
        g_seq_step = next_step;
    }
}

/* 联动工步执行函数 (非阻塞以 50ms 节拍轮询调用) */
static void Run_Sequence_Step_Handler(void)
{
    switch (g_seq_step)
    {
        case SYS_TASK_IDLE:
            break;
            
        case SYS_TASK_STEP_1_RAISE_SAFE:
        /* 工步1: 升降先缩回至安全提升高度 (20mm) */
            Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
            g_seq_step = SYS_TASK_STEP_2_OPEN_CLAW;
            break;
            
        case SYS_TASK_STEP_2_OPEN_CLAW:
            /* 等待升降到位，代表工步1彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
            /* 工步 1 完成，判定是否进行单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_2_OPEN_CLAW, 1);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，才立即发送爪子开和对准指令 */
                    Servo_App_SetTarget(&g_servo_claw, GRAB_CLAW_POS_OPEN, GRAB_CLAW_DURATION_MS);
                    Servo_App_SetTarget(&g_servo_align, g_grab_align_pos_start, GRAB_ALIGN_DURATION_MS);
                    g_seq_step = SYS_TASK_STEP_3_DESCEND_GRAB;
                }
            }
            break;
            
        case SYS_TASK_STEP_3_DESCEND_GRAB:
            /* 非阻塞等待两个舵机运动到位，代表工步2彻底完成 */
            if (Servo_App_IsTargetReached(&g_servo_claw) && Servo_App_IsTargetReached(&g_servo_align))
            {
            /* 工步 2 完成，判定单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_3_DESCEND_GRAB, 2);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，升降下降至预备抓取高度 (150mm) */
                    Stepper_App_MoveToPosition(ELEV_HEIGHT_GRAB, STEPPER_SPEED_ELEV);
                    g_seq_step = SYS_TASK_STEP_4_1_FIRST_DIG;
                }
            }
            break;
            
        case SYS_TASK_STEP_4_1_FIRST_DIG:
            /* 等待升降下降到位，代表工步3彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
            /* 工步 3 完成，判定单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_4_1_FIRST_DIG, 3);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，开始第一次半咬合（合拢至450），同时升降同步微下压挖掘 */
                    float target_height = ELEV_HEIGHT_GRAB + ELEV_FIRST_DIG_DEPTH;
                    Stepper_App_MoveToPosition(target_height, STEPPER_SPEED_ELEV);
                    Servo_App_SetTarget(&g_servo_claw, CLAW_MID_CLOSE_POS, GRAB_CLAW_DURATION_MS / 2);
                    g_seq_step = SYS_TASK_STEP_4_1_WAIT;
                }
            }
            break;
            
        case SYS_TASK_STEP_4_1_WAIT:
            /* 等待第一阶段的挖掘下压与半咬合到位 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM) && Servo_App_IsTargetReached(&g_servo_claw))
            {
            /* 向上回提（提起）8mm释放挤压应力，同时爪子微张（退回到350） */
                float target_height = ELEV_HEIGHT_GRAB + ELEV_FIRST_DIG_DEPTH - ELEV_RETRACT_HEIGHT;
                Stepper_App_MoveToPosition(target_height, STEPPER_SPEED_ELEV);
                Servo_App_SetTarget(&g_servo_claw, CLAW_MID_BACK_POS, GRAB_CLAW_DURATION_MS / 4);
                g_seq_step = SYS_TASK_STEP_4_2_WAIT;
            }
            break;
            
        case SYS_TASK_STEP_4_2_WAIT:
            /* 等待提起与微张动作结束 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM) && Servo_App_IsTargetReached(&g_servo_claw))
            {
            /* 动作到位后，设置 150ms 颗粒塌陷流动等待时间 */
                g_settle_delay_counter = 3;
                g_seq_step = SYS_TASK_STEP_4_2_DELAY;
            }
            break;
            
        case SYS_TASK_STEP_4_2_DELAY:
            /* 等待流动稳定 */
            if (g_settle_delay_counter == 0)
            {
            /* 向下全力深入挖掘压入15mm，同时爪子完全闭合咬死 (750) */
                float target_height = ELEV_HEIGHT_GRAB + ELEV_SECOND_DIG_DEPTH;
                Stepper_App_MoveToPosition(target_height, STEPPER_SPEED_ELEV);
                Servo_App_SetTarget(&g_servo_claw, GRAB_CLAW_POS_CLOSE, GRAB_CLAW_DURATION_MS);
                g_seq_step = SYS_TASK_STEP_4_3_WAIT;
            }
            break;
            
        case SYS_TASK_STEP_4_3_WAIT:
            /* 等待二次全力深入与咬死闭合到位 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM) && Servo_App_IsTargetReached(&g_servo_claw))
            {
            /* 咬合到位后，设置 1000ms 的抓取物理稳定延时 */
                g_sequence_settle_counter = 100;
                g_seq_step = SYS_TASK_STEP_4_3_SETTLE;
            }
            break;
            
        case SYS_TASK_STEP_4_3_SETTLE:
            /* 等待物理抓稳，代表工步4彻底完成 */
            if (g_sequence_settle_counter == 0)
            {
            /* 工步 4 完成，判定单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_5_RAISE_SAFE, 4);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，提升至安全搬运悬挂高度 (20mm) */
                    Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
                    g_seq_step = SYS_TASK_STEP_5_RAISE_SAFE;
                }
            }
            break;
            
        case SYS_TASK_STEP_5_RAISE_SAFE:
            /* 等待起吊升至安全高度，代表工步5彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
            /* 工步 5 完成，判定单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_6_ROTATE_TO_BOX, 5);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，底座水平旋转至货箱正上方 (600)，且抓斗对准 */
                    Servo_App_SetTarget(&g_servo_base, BASE_ROT_POS_BOX, BASE_ROT_DURATION_MS);
                    Servo_App_SetTarget(&g_servo_align, g_grab_align_pos_box, GRAB_ALIGN_DURATION_MS);
                    g_seq_step = SYS_TASK_STEP_6_ROTATE_TO_BOX;
                }
            }
            break;
            
        case SYS_TASK_STEP_6_ROTATE_TO_BOX:
            /* 等待旋转和角度对齐到位，代表工步6彻底完成 */
            if (Servo_App_IsTargetReached(&g_servo_base) && Servo_App_IsTargetReached(&g_servo_align))
            {
            /* 工步 6 完成，判定单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_7_DESCEND_DROP, 6);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，升降下放至货箱释放高度 (100mm) */
                    Stepper_App_MoveToPosition(ELEV_HEIGHT_DROP, STEPPER_SPEED_ELEV);
                    g_seq_step = SYS_TASK_STEP_7_DESCEND_DROP;
                }
            }
            break;
            
        case SYS_TASK_STEP_7_DESCEND_DROP:
            /* 等待下降到位，代表工步7彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
            /* 工步 7 完成，判定单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_8_RELEASE_CLAW, 7);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，爪子完全张开释放货物 (200) */
                    Servo_App_SetTarget(&g_servo_claw, GRAB_CLAW_POS_OPEN, GRAB_CLAW_DURATION_MS);
                    g_seq_step = SYS_TASK_STEP_8_RELEASE_CLAW;
                }
            }
            break;
            
        case SYS_TASK_STEP_8_RELEASE_CLAW:
            /* 等待爪子完全张开 */
            if (Servo_App_IsTargetReached(&g_servo_claw))
            {
            /* 设置放料落稳延时 800ms */
                g_sequence_settle_counter = 80;
                g_seq_step = SYS_TASK_STEP_8_WAIT_SETTLE;
            }
            break;
            
        case SYS_TASK_STEP_8_WAIT_SETTLE:
            /* 等待延时结束，代表工步8彻底完成 */
            if (g_sequence_settle_counter == 0)
            {
            /* 工步 8 完成，判定单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_9_RAISE_AFTER_RELEASE, 8);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，释放完毕，高度回缩起吊至安全高度 (20mm) */
                    Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
                    g_seq_step = SYS_TASK_STEP_9_RAISE_AFTER_RELEASE;
                }
            }
            break;
            
        case SYS_TASK_STEP_9_RAISE_AFTER_RELEASE:
            /* 等待升降安全撤回，代表工步9彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
            /* 工步 9 完成，判定单步阻断 */
                transition_to_next_step(SYS_TASK_STEP_10_RETURN_START, 9);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
            /* 连续运行下，底座及对齐回转复位 */
                    Servo_App_SetTarget(&g_servo_base, BASE_ROT_POS_START, BASE_ROT_DURATION_MS);
                    Servo_App_SetTarget(&g_servo_align, g_grab_align_pos_start, GRAB_ALIGN_DURATION_MS);
                    g_seq_step = SYS_TASK_STEP_10_RETURN_START;
                }
            }
            break;
            
        case SYS_TASK_STEP_10_RETURN_START:
            /* 等待复位对齐全部到位，代表工步10完成 */
            if (Servo_App_IsTargetReached(&g_servo_base) && Servo_App_IsTargetReached(&g_servo_align))
            {
            /* 搬运任务连续运行流程流程正常结束 */
                g_seq_step = SYS_TASK_IDLE;
                g_system_state = SYS_STATE_READY;
                printf(">> [流程结束]: 连续抓取搬运流程成功完成，系统已恢复 READY。\r\n");
            }
            break;
            
        default:
            g_seq_step = SYS_TASK_IDLE;
            g_system_state = SYS_STATE_READY;
            break;
    }
}

/**
 * @brief  系统状态机核心调度 Process
 */
void System_FSM_Process(void)
{
    static uint8_t  detect_idx = 1;     /* 当前正在探测的舵机序号(1, 2, 3) */
    static uint32_t telemetry_ticks = 0; /* 低频遥测轮询计时节拍 */
    
    /* 更新并驱动三个子舵机的状态机时间片 */
    Servo_App_Update(&g_servo_base);
    Servo_App_Update(&g_servo_align);
    Servo_App_Update(&g_servo_claw);
    
    /* 异常保护判定：如果在 READY 或 SEQUENCE 搬运中发生硬件故障，切入错误状态 */
    if (g_system_state == SYS_STATE_READY || g_system_state == SYS_STATE_RUNNING_SEQUENCE)
    {
        if (Servo_App_CheckAnyError() || Stepper_App_GetSystemState() == STEPPER_STATE_ERROR)
        {
            g_system_state = SYS_STATE_ERROR;
        }
    }
    
    switch (g_system_state)
    {
        case SYS_STATE_UNINIT:
            /* 1. 上电阶段，立即触发升降步进电机的非阻挡原点回零 */
            printf(">> [系统启动]: 启动步进升降电机碰撞回零...\r\n");
            Stepper_App_StartHoming();
            g_system_state = SYS_STATE_HOMING_STEPPER;
            break;
            
        case SYS_STATE_HOMING_STEPPER:
        {
            /* 2. 轮询等待步进回零状态 (PollHoming 内部非阻塞，不影响总线) */
            uint8_t homing_res = Stepper_App_PollHoming();
            if (homing_res == 1)
            {
                printf(">> [系统状态]: 升降回零成功。开始扫描总线舵机...\r\n");
                detect_idx = 1;
                HAL_Delay(300);
                g_system_state = SYS_STATE_DETECT_SERVOS;
            }
            else if (homing_res == 0)
            {
                /* 回零失败或超时 */
                printf(">> [系统状态]: 升降回零失败或超时。\r\n");
                g_system_state = SYS_STATE_ERROR;
            }
            break;
        }
            
        case SYS_STATE_DETECT_SERVOS:
            /* 3. 依次检测底座、对准、抓爪舵机是否在线 */
            {
                ServoDevice_t *p_dev = NULL;
                if (detect_idx == 1)      p_dev = &g_servo_base;
                else if (detect_idx == 2) p_dev = &g_servo_align;
                else if (detect_idx == 3) p_dev = &g_servo_claw;
                
                if (p_dev != NULL)
                {
                    trigger_servo_read_blocking_alternative(p_dev, SERIAL_SERVO_ID_READ, SYS_STATE_DETECT_SERVOS);
                }
                else
                {
                    /* 超过 3，说明全部检测完成，进入 READY 状态 */
                    g_system_state = SYS_STATE_READY;
                    telemetry_ticks = 0;
                    printf(">> [系统状态]: 自检扫描完成，系统处于 READY 状态。\n");
                }
            }
            break;
            
        case SYS_STATE_READY:
            /* 4. 闲置状态 */
            /* 50ms单位，每 20 次 (1000ms) 触发一次数据读取 and 全局异步更新 */
            telemetry_ticks++;
            if (telemetry_ticks >= 20)
            {
                telemetry_ticks = 0;
                
                /* 同步高度 */
                Stepper_App_TriggerPositionRead();
                
                /* 轮流获取底座、对准和抓爪的位置 */
                static uint8_t poll_idx = 1;
                ServoDevice_t *p_dev = NULL;
                if (poll_idx == 1)      p_dev = &g_servo_base;
                else if (poll_idx == 2) p_dev = &g_servo_align;
                else if (poll_idx == 3) p_dev = &g_servo_claw;
                
                if (p_dev != NULL)
                {
                    trigger_servo_read_blocking_alternative(p_dev, SERIAL_SERVO_POS_READ, SYS_STATE_READY);
                }
                
                poll_idx++;
                if (poll_idx > 3) poll_idx = 1;
            }
            break;
            
        case SYS_STATE_RUNNING_SEQUENCE:
            /* 5. 执行联动序列 */
            Run_Sequence_Step_Handler();
            
            /* 在执行序列过程中，可以根据节奏高频回读当前运动中的电机数据 */
            static uint32_t seq_read_ticks = 0;
            seq_read_ticks++;
            if (seq_read_ticks >= 2) /* 每 2 节拍 (100ms) */
            {
                seq_read_ticks = 0;
                /* 主动刷新一次当前的升降高度 */
                Stepper_App_TriggerPositionRead();
                
                /* 主动刷新一次当前运动舵机的位置 */
                if (g_servo_claw.state == SERVO_STATE_MOVING) {
                    trigger_servo_read_blocking_alternative(&g_servo_claw, SERIAL_SERVO_POS_READ, SYS_STATE_RUNNING_SEQUENCE);
                } else if (g_servo_base.state == SERVO_STATE_MOVING) {
                    trigger_servo_read_blocking_alternative(&g_servo_base, SERIAL_SERVO_POS_READ, SYS_STATE_RUNNING_SEQUENCE);
                } else if (g_servo_align.state == SERVO_STATE_MOVING) {
                    trigger_servo_read_blocking_alternative(&g_servo_align, SERIAL_SERVO_POS_READ, SYS_STATE_RUNNING_SEQUENCE);
                }
            }
            break;
            
        case SYS_STATE_WAIT_SERVO_REPLY:
            /* 6. 公共非阻塞等待响应逻辑 */
            if (g_serial_servo_controller.rx_completed)
            {
                g_serial_servo_controller.rx_completed = false;
                
            /* 从底层通信数据包解析，并刷入对应的舵机结构体对象中 */
                if (g_waiting_dev_ptr != NULL)
                {
                    uint8_t cmd = g_serial_servo_controller.rx_frame.elements.command;
                    uint8_t *args = g_serial_servo_controller.rx_frame.elements.args;
                    
                    g_waiting_dev_ptr->is_online = true;
                    g_waiting_dev_ptr->err_count = 0;
                    
                    if (cmd == SERIAL_SERVO_ID_READ)
                    {
                /* 探测握手包成功 */
                        if (g_saved_pre_state == SYS_STATE_DETECT_SERVOS)
                        {
                            const char *servo_name = "未知舵机";
                            if (g_waiting_dev_ptr == &g_servo_base)       servo_name = "底座旋转舵机";
                            else if (g_waiting_dev_ptr == &g_servo_align) servo_name = "抓斗对齐舵机";
                            else if (g_waiting_dev_ptr == &g_servo_claw)  servo_name = "抓爪开合舵机";
                            printf(">> [自检]: 检测到 %s 在线，物理ID: %d\r\n", servo_name, g_waiting_dev_ptr->id);
                            detect_idx++;
                        }
                    }
                    else if (cmd == SERIAL_SERVO_POS_READ)
                    {
                        g_waiting_dev_ptr->current_pos = (int16_t)BYTE_TO_HW(args[1], args[0]);
                    }
                    else if (cmd == SERIAL_SERVO_TEMP_READ)
                    {
                        g_waiting_dev_ptr->temp = args[0];
                    }
                    else if (cmd == SERIAL_SERVO_VIN_READ)
                    {
                        g_waiting_dev_ptr->vin = BYTE_TO_HW(args[1], args[0]);
                    }
                }
                
                /* 返回先前的状态继续工作 */
                g_system_state = g_saved_pre_state;
            }
                /* 软件计时器减到0，判定为通信超时丢失 (10ms) */
            else if (g_servo_reply_timeout_counter == 0)
            {
                if (g_waiting_dev_ptr != NULL)
                {
                    g_waiting_dev_ptr->err_count++;
                    if (g_saved_pre_state == SYS_STATE_DETECT_SERVOS)
                    {
                        const char *servo_name = "未知舵机";
                        if (g_waiting_dev_ptr == &g_servo_base)       servo_name = "底座旋转舵机";
                        else if (g_waiting_dev_ptr == &g_servo_align) servo_name = "抓斗对齐舵机";
                        else if (g_waiting_dev_ptr == &g_servo_claw)  servo_name = "抓爪开合舵机";
                        printf(">> [自检超时]: 检测 %s 失败，可能不在线，物理ID: %d\r\n", servo_name, g_waiting_dev_ptr->id);
                    }
                }
                
                /* 超时强制返回先前状态，防止总锁卡死主程序 */
                g_system_state = g_saved_pre_state;
            }
            break;
            
        case SYS_STATE_ERROR:
            /* 7. 紧急异常保护 */
            System_FSM_EmergencyStop();
            break;
    }
}

/**
 * @brief  一键触发执行全局起重搬运联合联动动作流程 (seq 指令)
 */
uint8_t System_FSM_StartSequence(void)
{
    if (g_system_state != SYS_STATE_READY)
    {
        return 0; /* 系统不处于就绪待命状态，拒绝执行 */
    }
    
    g_single_step_only = 0; /* 自动连续流转模式 */
    g_seq_step = SYS_TASK_STEP_1_RAISE_SAFE;
    g_system_state = SYS_STATE_RUNNING_SEQUENCE;
    printf(">> [连续启动]: 开始连续执行全局搬运联动动作序列...\r\n");
    return 1;
}

/**
 * @brief  执行单步工步调试流转 (step 指令)
 */
uint8_t System_FSM_StartSingleStep(uint8_t step_num)
{
    if (g_system_state != SYS_STATE_READY)
    {
        return 0; /* 系统未就绪拒绝执行 */
    }
    if (step_num < 1 || step_num > 10)
    {
        return 0; /* 有效范围 1 ~ 10 */
    }
    
    g_single_step_only = 1;
    g_system_state = SYS_STATE_RUNNING_SEQUENCE;
    
    g_next_single_step_num = step_num + 1;
    if (g_next_single_step_num > 10)
    {
        g_next_single_step_num = 1;
    }
    
    /* 强力分配单步启动任务步骤：直接发令，并在状态机中投入该动作的到位检测 */
    switch (step_num)
    {
        case 1:
            Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
            g_seq_step = SYS_TASK_STEP_2_OPEN_CLAW; /* 在步骤 2 等待到位 */
            break;
        case 2:
            Servo_App_SetTarget(&g_servo_claw, GRAB_CLAW_POS_OPEN, GRAB_CLAW_DURATION_MS);
            Servo_App_SetTarget(&g_servo_align, g_grab_align_pos_start, GRAB_ALIGN_DURATION_MS);
            g_seq_step = SYS_TASK_STEP_3_DESCEND_GRAB; /* 在步骤 3 等待到位 */
            break;
        case 3:
            Stepper_App_MoveToPosition(ELEV_HEIGHT_GRAB, STEPPER_SPEED_ELEV);
            g_seq_step = SYS_TASK_STEP_4_1_FIRST_DIG; /* 在步骤 4-1 等待到位 */
            break;
        case 4:
            {
                float target_height = ELEV_HEIGHT_GRAB + ELEV_FIRST_DIG_DEPTH;
                Stepper_App_MoveToPosition(target_height, STEPPER_SPEED_ELEV);
                Servo_App_SetTarget(&g_servo_claw, CLAW_MID_CLOSE_POS, GRAB_CLAW_DURATION_MS / 2);
                g_seq_step = SYS_TASK_STEP_4_1_WAIT; /* 等待第一阶段到位 */
            }
            break;
        case 5:
            Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
            g_seq_step = SYS_TASK_STEP_5_RAISE_SAFE; /* 在步骤 5 等待到位 */
            break;
        case 6:
            Servo_App_SetTarget(&g_servo_base, BASE_ROT_POS_BOX, BASE_ROT_DURATION_MS);
            Servo_App_SetTarget(&g_servo_align, g_grab_align_pos_box, GRAB_ALIGN_DURATION_MS);
            g_seq_step = SYS_TASK_STEP_6_ROTATE_TO_BOX; /* 在步骤 6 等待到位 */
            break;
        case 7:
            Stepper_App_MoveToPosition(ELEV_HEIGHT_DROP, STEPPER_SPEED_ELEV);
            g_seq_step = SYS_TASK_STEP_7_DESCEND_DROP; /* 在步骤 7 等待到位 */
            break;
        case 8:
            Servo_App_SetTarget(&g_servo_claw, GRAB_CLAW_POS_OPEN, GRAB_CLAW_DURATION_MS);
            g_seq_step = SYS_TASK_STEP_8_RELEASE_CLAW; /* 在步骤 8 等待到位及延时 */
            break;
        case 9:
            Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
            g_seq_step = SYS_TASK_STEP_9_RAISE_AFTER_RELEASE; /* 在步骤 9 等待到位 */
            break;
        case 10:
            Servo_App_SetTarget(&g_servo_base, BASE_ROT_POS_START, BASE_ROT_DURATION_MS);
            Servo_App_SetTarget(&g_servo_align, g_grab_align_pos_start, GRAB_ALIGN_DURATION_MS);
            g_seq_step = SYS_TASK_STEP_10_RETURN_START; /* 在步骤 10 等待到位并复位 */
            break;
    }
    
    printf(">> [单步启动]: 开始单步执行工步 %d ...\n", step_num);
    return 1;
}

/**
 * @brief  执行下一次顺序单步工步 (nextstep 指令触发)
 */
uint8_t System_FSM_StartNextSingleStep(void)
{
    uint8_t current_step = g_next_single_step_num;
    if (System_FSM_StartSingleStep(current_step) == 1)
    {
        return current_step;
    }
    return 0;
}

/**
 * @brief  供上位机调用：动态设置系统运行控制模式
 */
void System_FSM_SetControlMode(SystemControlMode_t mode)
{
    g_system_control_mode = mode;
    if (mode == SYS_MODE_AUTO)
    {
        printf(">> [系统模式]: 已动态切换至 [AUTO 自动运行模式]\r\n");
    }
    else
    {
        printf(">> [系统模式]: 已动态切换至 [MANUAL 手动调试模式]\r\n");
    }
}

/**
 * @brief  全局安全停转 (制动升降电机并彻底切断舵机力矩)
 */
void System_FSM_EmergencyStop(void)
{
    /* 立即发送步进刹车 */
    Stepper_App_EmergencyStop();
    
    /* 物理层切断三舵机供电解锁 */
    Servo_App_UnloadAll();
    
    g_system_state = SYS_STATE_ERROR;
}

/**
 * @brief  供上位机调用：动态微调更新对齐货箱时抓斗的对齐目标值
 */
void System_FSM_SetGrabAlignPos(uint16_t pos)
{
    if (pos > 1000) pos = 1000;
    g_grab_align_pos_box = pos;
    printf(">> [上位机输入]: 成功微调更新货箱对齐角度变量 g_grab_align_pos_box = %d\r\n", pos);
}

void System_FSM_SetGrabAlignStartPos(uint16_t pos)
{
    if (pos > 1000) pos = 1000;
    g_grab_align_pos_start = pos;
    printf(">> [对齐设置]: 成功更新初始对齐朝向 g_grab_align_pos_start = %d\r\n", pos);
}

/**
 * @brief  获取并生成全系统状态与各运行数据，供控制台 status 命令回显
 */
void System_FSM_GetStatusString(char *buf, uint16_t len)
{
    if (buf == NULL || len == 0) return;
    
    const char* state_str = "UNKNOWN";
    switch (g_system_state)
    {
        case SYS_STATE_UNINIT:           state_str = "SYS_UNINIT"; break;
        case SYS_STATE_HOMING_STEPPER:   state_str = "HOMING_STEPPER"; break;
        case SYS_STATE_DETECT_SERVOS:    state_str = "DETECT_SERVOS"; break;
        case SYS_STATE_READY:            state_str = "SYS_READY"; break;
        case SYS_STATE_RUNNING_SEQUENCE:  state_str = "RUNNING_SEQUENCE"; break;
        case SYS_STATE_WAIT_SERVO_REPLY: state_str = "WAIT_SERVO_REPLY"; break;
        case SYS_STATE_ERROR:            state_str = "SYS_ERROR"; break;
    }
    
    const char* step_str = "NONE";
    if (g_system_state == SYS_STATE_RUNNING_SEQUENCE)
    {
        switch (g_seq_step)
        {
            case SYS_TASK_STEP_1_RAISE_SAFE:         step_str = "1.RaiseSafe_Wait"; break;
            case SYS_TASK_STEP_2_OPEN_CLAW:          step_str = "2.OpenClaw_Wait"; break;
            case SYS_TASK_STEP_3_DESCEND_GRAB:       step_str = "3.DescendGrab_Wait"; break;
            case SYS_TASK_STEP_4_1_FIRST_DIG:        step_str = "4-1.FirstDig_Start"; break;
            case SYS_TASK_STEP_4_1_WAIT:             step_str = "4-1.FirstDig_Wait"; break;
            case SYS_TASK_STEP_4_2_RETRACT_SETTLE:   step_str = "4-2.RetractSettle_Start"; break;
            case SYS_TASK_STEP_4_2_WAIT:             step_str = "4-2.RetractSettle_Wait"; break;
            case SYS_TASK_STEP_4_2_DELAY:            step_str = "4-2.GranularFlow_Delay"; break;
            case SYS_TASK_STEP_4_3_SECOND_DIG_LOCK:  step_str = "4-3.SecondDig_Start"; break;
            case SYS_TASK_STEP_4_3_WAIT:             step_str = "4-3.SecondDig_Wait"; break;
            case SYS_TASK_STEP_4_3_SETTLE:           step_str = "4-3.CargoSettle_Wait"; break;
            case SYS_TASK_STEP_5_RAISE_SAFE:         step_str = "5.RaiseCargo_Wait"; break;
            case SYS_TASK_STEP_6_ROTATE_TO_BOX:      step_str = "6.RotateBox_Wait"; break;
            case SYS_TASK_STEP_7_DESCEND_DROP:       step_str = "7.DescendDrop_Wait"; break;
            case SYS_TASK_STEP_8_RELEASE_CLAW:       step_str = "8.ReleaseClaw_Wait"; break;
            case SYS_TASK_STEP_8_WAIT_SETTLE:        step_str = "8.DropSettle_Wait"; break;
            case SYS_TASK_STEP_9_RAISE_AFTER_RELEASE:step_str = "9.RetractArm_Wait"; break;
            case SYS_TASK_STEP_10_RETURN_START:      step_str = "10.ReturnReset_Wait"; break;
            default:                                 step_str = "RUNNING"; break;
        }
    }
    
    snprintf(buf, len,
                 "================= 起重器系统实时状态报告 =================\r\n"
                 "  [全局系统状态]: %s | [当前工步]: %s\r\n"
                 "  [调试运行模式]: %s | [初始对齐角度]: %d | [货箱对齐角度]: %d\r\n"
             "----------------------------------------------------------\r\n"
                 "  1. 升降高度(步进): %.2f mm\r\n"
                 "  2. 底座角度(舵机1): %d | 状态: %d | 在线: %s\r\n"
                 "  3. 对齐角度(舵机2): %d | 状态: %d | 在线: %s\r\n"
                 "  4. 爪子开合(舵机3): %d | 状态: %d | 在线: %s\r\n"
             "==========================================================\r\n",
             state_str, step_str,
             g_system_control_mode == SYS_MODE_AUTO ? "AUTO (自动)" : "MANUAL (手动)",
             g_grab_align_pos_start,
             g_grab_align_pos_box,
             Stepper_App_GetCurrentPosition(),
             g_servo_base.current_pos, g_servo_base.state, g_servo_base.is_online ? "YES" : "NO",
             g_servo_align.current_pos, g_servo_align.state, g_servo_align.is_online ? "YES" : "NO",
             g_servo_claw.current_pos, g_servo_claw.state, g_servo_claw.is_online ? "YES" : "NO");
}
