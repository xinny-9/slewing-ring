/**
 * *****************************************************************************
 * @file    app_system_fsm.c
 * @brief   起重器系统级主控制状态机 (Master FSM) 源文�
 * *****************************************************************************
 */

#include "app_system_fsm.h"
#include <stdio.h>
#include <string.h>

/* 定时器中�相关的全�标志与�减计数器声� */
volatile uint8_t  g_fsm_update_flag = 0;
volatile uint8_t  g_single_step_only = 0;
volatile uint32_t g_servo_reply_timeout_counter = 0;
volatile uint32_t g_sequence_settle_counter = 0;
volatile uint32_t g_settle_delay_counter = 0;

/* 上位机可�改的抓斗放��齐角度全局变量 */
volatile uint16_t g_grab_align_pos_box = GRAB_ALIGN_POS_BOX;

/* 系统状�机当前状� and �运序列工� */
static SystemState_t g_system_state = SYS_STATE_UNINIT;
static SystemTaskStep_t g_seq_step = SYS_TASK_IDLE;

/* 暂存等待响应前的状�与�标舵机��指� */
static SystemState_t g_saved_pre_state = SYS_STATE_UNINIT;
static ServoDevice_t *g_waiting_dev_ptr = NULL;
static uint8_t g_next_single_step_num = 1; /* 顺�单步调试下步指示器 */
volatile SystemControlMode_t g_system_control_mode = DEFAULT_SYS_MODE; /* 全局工作模式 */

/* 内部辅助函数：获取当前��的�标舵机实例指� */
static ServoDevice_t* get_servo_device_by_id(uint8_t id)
{
    if (id == SERVO_BASE_ROT)   return &g_servo_base;
    if (id == SERVO_GRAB_ALIGN) return &g_servo_align;
    if (id == SERVO_GRAB_CLAW)  return &g_servo_claw;
    return NULL;
}

/**
 * @brief  初�化起重器系统级联合状�机
 */
void System_FSM_Init(void)
{
    g_system_state = SYS_STATE_UNINIT;
    g_seq_step = SYS_TASK_IDLE;
    g_grab_align_pos_box = GRAB_ALIGN_POS_BOX;
    
    g_fsm_update_flag = 0;
    g_single_step_only = 0;
    g_servo_reply_timeout_counter = 0;
    g_sequence_settle_counter = 0;
    g_settle_delay_counter = 0;
    
    g_waiting_dev_ptr = NULL;
    g_next_single_step_num = 1;
    g_system_control_mode = DEFAULT_SYS_MODE;
    
    /* 调用并初始化子层舵机结构� */
    Servo_App_Init();
}

/**
 * @brief  向指定的舵机发起位置异��取请求，并暂时切换状�机以进行非阻�等�
 */
static void trigger_servo_read_blocking_alternative(ServoDevice_t *dev, uint8_t cmd, SystemState_t current_state)
{
    g_waiting_dev_ptr = dev;
    g_saved_pre_state = current_state;
    
    /* 1. 发起非阻塞�取请求 (�管发送，不原� while 等待) */
    Servo_App_TriggerRead(dev, cmd);
    
    /* 2. 设置 10ms 通信等待超时计时� (50ms 周期� 10ms 足�响�) */
    g_servo_reply_timeout_counter = 1; 
    
    /* 3. 切换状�至非阻塞等待响� */
    g_system_state = SYS_STATE_WAIT_SERVO_REPLY;
}

/**
 * @brief  过渡跳转辅助函数（在单�调试模式下起到阻断�动跳�的作��
 * @param  next_step: 连续模式下的下一步�
 * @param  current_step_num: 当前步�的数字序号 (1 ~ 9)
 */
static void transition_to_next_step(SystemTaskStep_t next_step, uint8_t current_step_num)
{
    /* 如果�调试单�指令触发，或是处于手动模式，到位后�律安全暂停拦� */
    if (g_single_step_only || g_system_control_mode == SYS_MODE_MANUAL)
    {
        g_seq_step = SYS_TASK_IDLE;
        g_system_state = SYS_STATE_READY;
        printf(">> [系统暂停]: � %d 步动作到位，系统已安全暂停并回归 READY。\r\n", current_step_num);
    }
    else
    {
        /* �动模式下，自动流�到下�� */
        g_seq_step = next_step;
    }
}

/* 联动工�执行函� (非阻塞以 50ms 节拍�询调�) */
static void Run_Sequence_Step_Handler(void)
{
    switch (g_seq_step)
    {
        case SYS_TASK_IDLE:
            break;
            
        case SYS_TASK_STEP_1_RAISE_SAFE:
            /* 步�1: 升降先缩回至安全提升高度 (20mm) */
            Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
            g_seq_step = SYS_TASK_STEP_2_OPEN_CLAW;
            break;
            
        case SYS_TASK_STEP_2_OPEN_CLAW:
            /* 等待升降到位，代表��1彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
                /* 步� 1 完成，判定是否进行单步阻� */
                transition_to_next_step(SYS_TASK_STEP_2_OPEN_CLAW, 1);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，才立即发�爪子开和�准指令 */
                    Servo_App_SetTarget(&g_servo_claw, GRAB_CLAW_POS_OPEN, GRAB_CLAW_DURATION_MS);
                    Servo_App_SetTarget(&g_servo_align, GRAB_ALIGN_POS_START, GRAB_ALIGN_DURATION_MS);
                    g_seq_step = SYS_TASK_STEP_3_DESCEND_GRAB;
                }
            }
            break;
            
        case SYS_TASK_STEP_3_DESCEND_GRAB:
            /* 非阻塞等待两�舵机运动到位，代表��2彻底完成 */
            if (Servo_App_IsTargetReached(&g_servo_claw) && Servo_App_IsTargetReached(&g_servo_align))
            {
                /* 步� 2 完成，判定单步阻� */
                transition_to_next_step(SYS_TASK_STEP_3_DESCEND_GRAB, 2);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，升降下降至预�抓取高� (150mm) */
                    Stepper_App_MoveToPosition(ELEV_HEIGHT_GRAB, STEPPER_SPEED_ELEV);
                    g_seq_step = SYS_TASK_STEP_4_1_FIRST_DIG;
                }
            }
            break;
            
        case SYS_TASK_STEP_4_1_FIRST_DIG:
            /* 等待升降下降到位，代表��3彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
                /* 步� 3 完成，判定单步阻� */
                transition_to_next_step(SYS_TASK_STEP_4_1_FIRST_DIG, 3);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，开始��次半�合（合拢�450），同时升降同�微下压挖掘（ELEV_FIRST_DIG_DEPTH=6mm� */
                    float target_height = ELEV_HEIGHT_GRAB + ELEV_FIRST_DIG_DEPTH;
                    Stepper_App_MoveToPosition(target_height, STEPPER_SPEED_ELEV);
                    Servo_App_SetTarget(&g_servo_claw, CLAW_MID_CLOSE_POS, GRAB_CLAW_DURATION_MS / 2);
                    g_seq_step = SYS_TASK_STEP_4_1_WAIT;
                }
            }
            break;
            
        case SYS_TASK_STEP_4_1_WAIT:
            /* 等待��阶�的挖掘下压与半�合到� */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM) && Servo_App_IsTargetReached(&g_servo_claw))
            {
                /* 向上回���8mm释放挤压应力，同时爪子微张（�回到350� */
                float target_height = ELEV_HEIGHT_GRAB + ELEV_FIRST_DIG_DEPTH - ELEV_RETRACT_HEIGHT;
                Stepper_App_MoveToPosition(target_height, STEPPER_SPEED_ELEV);
                Servo_App_SetTarget(&g_servo_claw, CLAW_MID_BACK_POS, GRAB_CLAW_DURATION_MS / 4);
                g_seq_step = SYS_TASK_STEP_4_2_WAIT;
            }
            break;
            
        case SYS_TASK_STEP_4_2_WAIT:
            /* 等待�起与�张动作结� */
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
                /* 向下全力深入挖掘压入15mm，同时爪子完全闭合咬� (750) */
                float target_height = ELEV_HEIGHT_GRAB + ELEV_SECOND_DIG_DEPTH;
                Stepper_App_MoveToPosition(target_height, STEPPER_SPEED_ELEV);
                Servo_App_SetTarget(&g_servo_claw, GRAB_CLAW_POS_CLOSE, GRAB_CLAW_DURATION_MS);
                g_seq_step = SYS_TASK_STEP_4_3_WAIT;
            }
            break;
            
        case SYS_TASK_STEP_4_3_WAIT:
            /* 等待二�全力深入与�死闭合到� */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM) && Servo_App_IsTargetReached(&g_servo_claw))
            {
                /* �合到位后，�置 1000ms 的抓取物理稳定延� */
                g_sequence_settle_counter = 100;
                g_seq_step = SYS_TASK_STEP_4_3_SETTLE;
            }
            break;
            
        case SYS_TASK_STEP_4_3_SETTLE:
            /* 等待物理抓稳，代表��4彻底完成 */
            if (g_sequence_settle_counter == 0)
            {
                /* 步� 4 完成，判定单步阻� */
                transition_to_next_step(SYS_TASK_STEP_5_RAISE_SAFE, 4);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，提升至安全�运悬挂高� (20mm) */
                    Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
                    g_seq_step = SYS_TASK_STEP_5_RAISE_SAFE;
                }
            }
            break;
            
        case SYS_TASK_STEP_5_RAISE_SAFE:
            /* 等待起吊升至安全高度，代表��5彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
                /* 步� 5 完成，判定单步阻� */
                transition_to_next_step(SYS_TASK_STEP_6_ROTATE_TO_BOX, 5);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，底座水平旋�至货箱�上� (600)，且抓斗对准 */
                    Servo_App_SetTarget(&g_servo_base, BASE_ROT_POS_BOX, BASE_ROT_DURATION_MS);
                    Servo_App_SetTarget(&g_servo_align, g_grab_align_pos_box, GRAB_ALIGN_DURATION_MS);
                    g_seq_step = SYS_TASK_STEP_6_ROTATE_TO_BOX;
                }
            }
            break;
            
        case SYS_TASK_STEP_6_ROTATE_TO_BOX:
            /* 等待旋转和�度对齐到位，代表��6彻底完成 */
            if (Servo_App_IsTargetReached(&g_servo_base) && Servo_App_IsTargetReached(&g_servo_align))
            {
                /* 步� 6 完成，判定单步阻� */
                transition_to_next_step(SYS_TASK_STEP_7_DESCEND_DROP, 6);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，升降下放至货�释放高� (100mm) */
                    Stepper_App_MoveToPosition(ELEV_HEIGHT_DROP, STEPPER_SPEED_ELEV);
                    g_seq_step = SYS_TASK_STEP_7_DESCEND_DROP;
                }
            }
            break;
            
        case SYS_TASK_STEP_7_DESCEND_DROP:
            /* 等待下降到位，代表��7彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
                /* 步� 7 完成，判定单步阻� */
                transition_to_next_step(SYS_TASK_STEP_8_RELEASE_CLAW, 7);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，爪子完全张�释放货物 (200) */
                    Servo_App_SetTarget(&g_servo_claw, GRAB_CLAW_POS_OPEN, GRAB_CLAW_DURATION_MS);
                    g_seq_step = SYS_TASK_STEP_8_RELEASE_CLAW;
                }
            }
            break;
            
        case SYS_TASK_STEP_8_RELEASE_CLAW:
            /* 等待�子完全张� */
            if (Servo_App_IsTargetReached(&g_servo_claw))
            {
                /* 设置放料落稳延时 800ms */
                g_sequence_settle_counter = 80;
                g_seq_step = SYS_TASK_STEP_8_WAIT_SETTLE;
            }
            break;
            
        case SYS_TASK_STEP_8_WAIT_SETTLE:
            /* 等待延时结束，代表��8彻底完成 */
            if (g_sequence_settle_counter == 0)
            {
                /* 步� 8 完成，判定单步阻� */
                transition_to_next_step(SYS_TASK_STEP_9_RAISE_AFTER_RELEASE, 8);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，释放完毕，高度回缩起吊至安全高� (20mm) */
                    Stepper_App_MoveToPosition(ELEV_HEIGHT_SAFE, STEPPER_SPEED_ELEV);
                    g_seq_step = SYS_TASK_STEP_9_RAISE_AFTER_RELEASE;
                }
            }
            break;
            
        case SYS_TASK_STEP_9_RAISE_AFTER_RELEASE:
            /* 等待升降安全撤回，代表��9彻底完成 */
            if (Stepper_App_IsTargetReached(TOLERANCE_STEPPER_MM))
            {
                /* 步� 9 完成，判定单步阻� */
                transition_to_next_step(SYS_TASK_STEP_10_RETURN_START, 9);
                
                if (g_seq_step != SYS_TASK_IDLE)
                {
                    /* 连续运�下，底座及对准回转复位 */
                    Servo_App_SetTarget(&g_servo_base, BASE_ROT_POS_START, BASE_ROT_DURATION_MS);
                    Servo_App_SetTarget(&g_servo_align, GRAB_ALIGN_POS_START, GRAB_ALIGN_DURATION_MS);
                    g_seq_step = SYS_TASK_STEP_10_RETURN_START;
                }
            }
            break;
            
        case SYS_TASK_STEP_10_RETURN_START:
            /* 等待复位对准全部到位，代表��10完成 */
            if (Servo_App_IsTargetReached(&g_servo_base) && Servo_App_IsTargetReached(&g_servo_align))
            {
                /* �运任务连�运�流程�常结束 */
                g_seq_step = SYS_TASK_IDLE;
                g_system_state = SYS_STATE_READY;
                printf(">> [流程结束]: 连续抓取�运流程成功完成，系统已恢� READY。\r\n");
            }
            break;
            
        default:
            g_seq_step = SYS_TASK_IDLE;
            g_system_state = SYS_STATE_READY;
            break;
    }
}

/**
 * @brief  系统状�机核心调度 Process
 */
void System_FSM_Process(void)
{
    static uint8_t  detect_idx = 1;     /* 当前正在探测的舵机序�(1, 2, 3) */
    static uint32_t telemetry_ticks = 0; /* 低�遥测轮询�时节拍 */
    
    /* 更新并驱动三�子舵机的状�机时间� */
    Servo_App_Update(&g_servo_base);
    Servo_App_Update(&g_servo_align);
    Servo_App_Update(&g_servo_claw);
    
    /* 异常保护判定：�果� READY � SEQUENCE �运中发生�件故障，切入错�� */
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
            /* 1. 上电阶�，立即触发升降步进电机的非阻�原点回� */
            printf(">> [系统�动]: �动�进升降电机碰撞回零...\r\n");
            Stepper_App_StartHoming();
            g_system_state = SYS_STATE_HOMING_STEPPER;
            break;
            
        case SYS_STATE_HOMING_STEPPER:
        {
            /* 2. �询等待�进回零状� (PollHoming 内部非阻塞，不影响�线) */
            uint8_t homing_res = Stepper_App_PollHoming();
            if (homing_res == 1)
            {
                printf(">> [系统状�]: 升降回零成功。开始扫描�线舵机...\r\n");
                detect_idx = 1;
                g_system_state = SYS_STATE_DETECT_SERVOS;
            }
            else if (homing_res == 0)
            {
                /* 回零失败或超� */
                g_system_state = SYS_STATE_ERROR;
            }
            break;
        }
            
        case SYS_STATE_DETECT_SERVOS:
            /* 3. 逐个非阻塞�取舵机 ID，判��否全部在� */
            if (detect_idx <= 3)
            {
                ServoDevice_t *p_dev = get_servo_device_by_id(detect_idx);
                if (p_dev != NULL)
                {
                    trigger_servo_read_blocking_alternative(p_dev, SERIAL_SERVO_ID_READ, SYS_STATE_DETECT_SERVOS);
                }
            }
            else
            {
                /* 三个舵机均握手成功在线，进入待命就绪状� */
                g_system_state = SYS_STATE_READY;
                telemetry_ticks = 0;
                printf(">> [系统就绪]: 升降电机与三�舵机健康��完毕，READY 待命�。\r\n");
            }
            break;
            
        case SYS_STATE_READY:
            /* 4. 空闲待命模式 */
            /* 50ms节拍下，� 20 节拍 (1000ms) 触发�次数�回�更新，完全非阻塞异� */
            telemetry_ticks++;
            if (telemetry_ticks >= 20)
            {
                telemetry_ticks = 0;
                
                /* 主动发出读指令，回传的数�会在��回调�� ParseFrame 解析并刷新给结构� */
                Stepper_App_TriggerPositionRead();
                
                /* �询��线舵机的位�、电压和温度 */
                static uint8_t poll_servo_id = 1;
                ServoDevice_t *p_dev = get_servo_device_by_id(poll_servo_id);
                if (p_dev != NULL)
                {
                    trigger_servo_read_blocking_alternative(p_dev, SERIAL_SERVO_POS_READ, SYS_STATE_READY);
                }
                
                poll_servo_id++;
                if (poll_servo_id > 3) poll_servo_id = 1;
            }
            break;
            
        case SYS_STATE_RUNNING_SEQUENCE:
            /* 5. 执�联动序� */
            Run_Sequence_Step_Handler();
            
            /* 在执行序列过程中，可以根�节�高频回读当前运动中的电机数� */
            static uint32_t seq_read_ticks = 0;
            seq_read_ticks++;
            if (seq_read_ticks >= 2) /* � 2 节拍 (100ms) */
            {
                seq_read_ticks = 0;
                /* 主动刷新�次当前的升降高度 */
                Stepper_App_TriggerPositionRead();
                
                /* 主动刷新�次当前运动舵机的位置 */
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
            /* 6. �共非阻�等待响应�辑 */
            if (g_serial_servo_controller.rx_completed)
            {
                g_serial_servo_controller.rx_completed = false;
                
                /* 从底层�信数据包解析，并刷入�应的舵机结构体对象� */
                if (g_waiting_dev_ptr != NULL)
                {
                    uint8_t cmd = g_serial_servo_controller.rx_frame.elements.command;
                    uint8_t *args = g_serial_servo_controller.rx_frame.elements.args;
                    
                    g_waiting_dev_ptr->is_online = true;
                    g_waiting_dev_ptr->err_count = 0;
                    
                    if (cmd == SERIAL_SERVO_ID_READ)
                    {
                        /* 探测握手包成� */
                        if (g_saved_pre_state == SYS_STATE_DETECT_SERVOS)
                        {
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
                
                /* 返回先前的状态继�工作 */
                g_system_state = g_saved_pre_state;
            }
            /* �件定时器减到0，判定为通信超时丢失 (10ms) */
            else if (g_servo_reply_timeout_counter == 0)
            {
                if (g_waiting_dev_ptr != NULL)
                {
                    g_waiting_dev_ptr->err_count++;
                }
                
                /* 超时强制�回先前状态，防��锁卡�主程序 */
                g_system_state = g_saved_pre_state;
            }
            break;
            
        case SYS_STATE_ERROR:
            /* 7. 紧�异常保� */
            System_FSM_EmergencyStop();
            break;
    }
}

/**
 * @brief  ��触发执�全�起重�运联合联动动作流� (seq 指令)
 */
uint8_t System_FSM_StartSequence(void)
{
    if (g_system_state != SYS_STATE_READY)
    {
        return 0; /* 系统不�于就绪待命状�，拒绝执� */
    }
    
    g_single_step_only = 0; /* �动连�流转模式 */
    g_seq_step = SYS_TASK_STEP_1_RAISE_SAFE;
    g_system_state = SYS_STATE_RUNNING_SEQUENCE;
    printf(">> [连续�动]: �始连�执�全��运联动动作序�...\r\n");
    return 1;
}

/**
 * @brief  执�单步工步调试流� (step 指令)
 */
uint8_t System_FSM_StartSingleStep(uint8_t step_num)
{
    if (g_system_state != SYS_STATE_READY)
    {
        return 0; /* 系统不�于就绪待命状�，拒绝执� */
    }
    if (step_num < 1 || step_num > 10)
    {
        return 0; /* 步�号错�，有效范� 1 ~ 10 */
    }
    
    g_single_step_only = 1; /* ��单�调试拦� */
    g_system_state = SYS_STATE_RUNNING_SEQUENCE;
    
    /* 同�更新顺序单步序号为下一� */
    g_next_single_step_num = step_num + 1;
    if (g_next_single_step_num > 10)
    {
        g_next_single_step_num = 1;
    }
    
    /* 强�根�上位机�求的单步号，跳�到�应的状态起� */
    switch (step_num)
    {
        case 1:  g_seq_step = SYS_TASK_STEP_1_RAISE_SAFE; break;
        case 2:  g_seq_step = SYS_TASK_STEP_2_OPEN_CLAW; break;
        case 3:  g_seq_step = SYS_TASK_STEP_3_DESCEND_GRAB; break;
        case 4:  g_seq_step = SYS_TASK_STEP_4_1_FIRST_DIG; break;
        case 5:  g_seq_step = SYS_TASK_STEP_5_RAISE_SAFE; break;
        case 6:  g_seq_step = SYS_TASK_STEP_6_ROTATE_TO_BOX; break;
        case 7:  g_seq_step = SYS_TASK_STEP_7_DESCEND_DROP; break;
        case 8:  g_seq_step = SYS_TASK_STEP_8_RELEASE_CLAW; break;
        case 9:  g_seq_step = SYS_TASK_STEP_9_RAISE_AFTER_RELEASE; break;
        case 10: g_seq_step = SYS_TASK_STEP_10_RETURN_START; break;
    }
    
    printf(">> [单�启动]: �始单步执行�� %d ...\r\n", step_num);
    return 1;
}

/**
 * @brief  执�下�次顺次单步工� (nextstep 指令)
 */
uint8_t System_FSM_StartNextSingleStep(void)
{
    uint8_t current_step = g_next_single_step_num;
    if (System_FSM_StartSingleStep(current_step) == 1)
    {
        /* �动成功后，�算下一�应�执行的工�序� */
        g_next_single_step_num++;
        if (g_next_single_step_num > 10)
        {
            g_next_single_step_num = 1;
        }
        return current_step;
    }
    return 0;
}

/**
 * @brief  供上位机调用：动态�置系统运�控制模�
 */
void System_FSM_SetControlMode(SystemControlMode_t mode)
{
    g_system_control_mode = mode;
    if (mode == SYS_MODE_AUTO)
    {
        printf(">> [系统模式]: 已动态切�� [AUTO �动运行模式]\r\n");
    }
    else
    {
        printf(">> [系统模式]: 已动态切�� [MANUAL 手动调试模式]\r\n");
    }
}

/**
 * @brief  全局安全紧�停� (制动升降电机并彻底切�舵机力矩)
 */
void System_FSM_EmergencyStop(void)
{
    /* 立即发��进刹车 */
    Stepper_App_EmergencyStop();
    
    /* 物理层切�三舵机供电解� */
    Servo_App_UnloadAll();
    
    g_system_state = SYS_STATE_ERROR;
}

/**
 * @brief  供上位机调用：动态微调更新�准货�时抓斗的�齐�标�
 */
void System_FSM_SetGrabAlignPos(uint16_t pos)
{
    if (pos > 1000) pos = 1000;
    g_grab_align_pos_box = pos;
    printf(">> [上位机输�]: 成功�调更新货箱�齐角度变量 g_grab_align_pos_box = %d\r\n", pos);
}

/**
 * @brief  获取并生成全系统状�与各机构的运�数�，供控制� status 命令回显
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
             "================= 起重器系统实时状态报� =================\r\n"
             "  [全局系统状�]: %s | [当前工�]: %s\r\n"
             "  [调试运�模式]: %s | [货��齐角度]: %d\r\n"
             "----------------------------------------------------------\r\n"
             "  1. 升降高度(步进): %.2f mm\r\n"
             "  2. 底座角度(舵机1): %d | 状�: %d | 在线: %s\r\n"
             "  3. 对齐角度(舵机2): %d | 状�: %d | 在线: %s\r\n"
             "  4. �子开�(舵机3): %d | 状�: %d | 在线: %s\r\n"
             "==========================================================\r\n",
             state_str, step_str,
             g_system_control_mode == SYS_MODE_AUTO ? "AUTO (�Զ�)" : "MANUAL (�ֶ�)",
             g_grab_align_pos_box,
             Stepper_App_GetCurrentPosition(),
             g_servo_base.current_pos, g_servo_base.state, g_servo_base.is_online ? "YES" : "NO",
             g_servo_align.current_pos, g_servo_align.state, g_servo_align.is_online ? "YES" : "NO",
             g_servo_claw.current_pos, g_servo_claw.state, g_servo_claw.is_online ? "YES" : "NO");
}
