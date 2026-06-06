/**

 * *****************************************************************************

 * @file    app_stepper_ctrl.c

 * @brief   丝杆步进电机应用层控制模块实现 (基于 Emm_V5 驱动库)

 * *****************************************************************************

 */



#include "app_stepper_ctrl.h"

#include <stdio.h>



/* 电机及状态全局变量声明 */

static Emm_V5_Motor g_app_stepper;

static StepperSysState_t g_system_state = STEPPER_STATE_UNINIT;
static uint32_t g_homing_start_time = 0;       /* 记录回零动作开始的时间戳 */
static float g_target_pos_mm = 0.0f;
volatile uint8_t g_has_started_homing = 0;           /* 记录电机的目标绝对位置 (mm) */



/**

  * @brief    应用层丝杆电机系统初始化

  */

void Stepper_App_Init(UART_HandleTypeDef *huart, uint8_t addr)

{

    /* 1. 初始化底层的 Emm_V5 驱动句柄 */

    Emm_V5_Init(&g_app_stepper, huart, addr);
    
    /* 强制写入细分配置为 16 细分，并保存至 EEPROM，防止手动误改导致距离失准 */
    Emm_V5_Modify_Subdivision(&g_app_stepper, true, STEPPER_SUBDIVISION);
    HAL_Delay(150);

    

    /* 2. 软件状态机置为未初始化 */

    g_system_state = STEPPER_STATE_UNINIT;

    

    /* 3. 延时等待电机控制板供电完成 */

    HAL_Delay(500);

    

    /* 4. 配置电机的无限位碰撞回零底层参数 (写入电机 EEPROM 永久存储)

     * 参数含义：

     *   - 模式2：多圈无限位碰撞回零 (堵转回零)

     *   - 方向1：CW (顺时针方向，即滑块朝电机轴心一端回缩)

     *   - 回零速度：20 RPM (更低速撞击，保护丝杆)

     *   - 回零超时：12000 ms (12秒超时退出)

     *   - 判定转速：8 RPM (当转速受阻降到15以下时)

     *   - 判定电流：200 mA (降低电流阈值以减小撞击力矩) (当堵转电流高于350mA时)

     *   - 判定时间：100 ms (反应更灵敏) (以上卡死条件持续200ms则判定到位)

     *   - 上电不自启：false

     */

    Emm_V5_Origin_Modify_Params(&g_app_stepper, true, HOMING_MODE, HOMING_DIR, HOMING_SPEED_RPM, HOMING_TIMEOUT_MS, HOMING_SL_VEL_RPM, HOMING_SL_CUR_MA, HOMING_SL_TIME_MS, HOMING_AUTO_START);

    

    /* 5. 稍微延时，确保电机配置保存完毕 */

    HAL_Delay(150);

}



/**

  * @brief    执行上电碰撞自动寻原点流程 (阻塞查询方式)

  */


uint8_t Stepper_App_ExecuteHoming(void)
{
    g_system_state = STEPPER_STATE_HOMING;
    g_has_started_homing = 0;
    
    /* 1. 确认使能 */
    Emm_V5_En_Control(&g_app_stepper, true, false);
    HAL_Delay(100);
    
    /* 2. 触发回零 (模式2) */
    g_app_stepper.origin_state = 0xFF;
    Emm_V5_Origin_Trigger_Return(&g_app_stepper, 2, false);
    
    /* 3. 循环等待回零状态 */
    uint32_t start_time = HAL_GetTick();
    
    while (1)
    {
        HAL_Delay(250); /* 250ms 查询一次 */
        Emm_V5_Read_Sys_Params(&g_app_stepper, S_ORG);
        HAL_Delay(50);
        printf(">> Homing poll, current state = 0x%02X\r\n", g_app_stepper.origin_state);
        
        /* 判定电机确实已经开动回零中 (Bit 2 变为 1, 对应 0x07) */
        if (g_app_stepper.origin_state != 0xFF && (g_app_stepper.origin_state & 0x04) == 0x04)
        {
            g_has_started_homing = 1;
        }
        
        /* 成功条件：已被检测到正在回零，且 Bit 2 变回了 0 (代表回零结束并停止) */
        if (g_has_started_homing && g_app_stepper.origin_state != 0xFF && (g_app_stepper.origin_state & 0x04) == 0)
        {
            g_has_started_homing = 0;
            
            Emm_V5_Reset_CurPos_To_Zero(&g_app_stepper);
            HAL_Delay(100);
            
            // 安全反向回缩 (顺时针 CW)
            uint32_t back_pulses = (uint32_t)((SAFE_CLEARANCE_MM / SCREW_LEAD_MM) * PULSE_PER_ROUND);
            Emm_V5_Pos_Control(&g_app_stepper, EMM_CW, 500, 10, back_pulses, false, false);
            HAL_Delay(800); 
            
            Emm_V5_Reset_CurPos_To_Zero(&g_app_stepper);
            HAL_Delay(100);
            
            g_system_state = STEPPER_STATE_READY;
            g_target_pos_mm = 0.0f;
            return 1;
        }
        
        /* 失败 */
        if (g_app_stepper.origin_state == 0)
        {
            g_has_started_homing = 0;
            g_system_state = STEPPER_STATE_ERROR;
            return 0;
        }
        
        /* 超时 */
        if (HAL_GetTick() - start_time > 15000)
        {
            g_has_started_homing = 0;
            Emm_V5_Origin_Interrupt(&g_app_stepper);
            g_system_state = STEPPER_STATE_ERROR;
            return 0;
        }
    }
}

/**
  * @brief    异步回零触发 (主控状态机调度)
  */
void Stepper_App_StartHoming(void)
{
    g_system_state = STEPPER_STATE_HOMING;
    g_has_started_homing = 0; // 重置归零开动标记锁
    
    /* 1. 确认使能 */
    Emm_V5_En_Control(&g_app_stepper, true, false);
    HAL_Delay(100);
    
    /* 2. 在触发前再次下发回零配置，使用 false 避免 Flash 写入忙碌 */
    Emm_V5_Origin_Modify_Params(&g_app_stepper, false, HOMING_MODE, HOMING_DIR, HOMING_SPEED_RPM, HOMING_TIMEOUT_MS, HOMING_SL_VEL_RPM, HOMING_SL_CUR_MA, HOMING_SL_TIME_MS, HOMING_AUTO_START);
    HAL_Delay(150);
    
    /* 3. 触发回零 (模式2) */
    g_app_stepper.origin_state = 0xFF;
    Emm_V5_Origin_Trigger_Return(&g_app_stepper, 2, false);
    
    /* 4. 延时 300ms 让电机先开动并清除旧的完成状态 */
    HAL_Delay(300);
    
    /* 5. 记录开始时间 */
    g_homing_start_time = HAL_GetTick();
}

/**
  * @brief    轮询查询回零状态 (主控状态机轮询)
  */
uint8_t Stepper_App_PollHoming(void)
{
    static uint32_t last_poll_time = 0;
    uint32_t now = HAL_GetTick();
    
    /* 每 250ms 查询一次电机回零状态 */
    if (now - last_poll_time >= 250)
    {
        last_poll_time = now;
        Emm_V5_Read_Sys_Params(&g_app_stepper, S_ORG);
    }
    
    /* 判定电机确实已经开动回零中 (Bit 2 变为 1, 对应 0x07) */
    if (g_app_stepper.origin_state != 0xFF && (g_app_stepper.origin_state & 0x04) == 0x04)
    {
        g_has_started_homing = 1;
    }
    
    /* 成功条件：已被检测到正在回零，且 Bit 2 变回了 0 (代表回零结束并停止) */
    if (g_has_started_homing && g_app_stepper.origin_state != 0xFF && (g_app_stepper.origin_state & 0x04) == 0)
    {
        g_has_started_homing = 0; // 重置标记锁
        
        Emm_V5_Reset_CurPos_To_Zero(&g_app_stepper);
        HAL_Delay(100);
        
        // 安全反向回缩 (顺时针 CW)
        uint32_t back_pulses = (uint32_t)((SAFE_CLEARANCE_MM / SCREW_LEAD_MM) * PULSE_PER_ROUND);
        Emm_V5_Pos_Control(&g_app_stepper, EMM_CW, 500, 10, back_pulses, false, false);
        HAL_Delay(800);
        
        Emm_V5_Reset_CurPos_To_Zero(&g_app_stepper);
        HAL_Delay(100);
        
        g_system_state = STEPPER_STATE_READY;
        g_target_pos_mm = 0.0f;
        return 1; /* 返回成功 */
    }
    
    /* 失败 */
    if (g_app_stepper.origin_state == 0)
    {
        g_has_started_homing = 0;
        g_system_state = STEPPER_STATE_ERROR;
        return 0; /* 返回失败 */
    }
    
    /* 15 秒超时判定 */
    if (now - g_homing_start_time > 15000)
    {
        g_has_started_homing = 0;
        Emm_V5_Origin_Interrupt(&g_app_stepper);
        g_system_state = STEPPER_STATE_ERROR;
        return 0; /* 返回超时 */
    }
    
    return 2; /* 正在回零中 */
}

bool Stepper_App_IsTargetReached(float tolerance_mm)
{
    float current = Stepper_App_GetCurrentPosition();
    if (current < 0)
    {
        return false;
    }
    
    float diff = current - g_target_pos_mm;
    if (diff < 0) diff = -diff;
    
    if (diff <= tolerance_mm)
    {
        return true;
    }
    return false;
}

/**
  * @brief    移动到丝杠行程的绝对位置 (单位: mm)
  */
uint8_t Stepper_App_MoveToPosition(float position_mm, uint16_t speed_rpm)
{
    g_target_pos_mm = position_mm;
    
    if (g_system_state != STEPPER_STATE_READY)
    {
        return 0;
    }
    
    if (position_mm < 0.0f || position_mm > SCREW_MAX_TRAVEL_MM)
    {
        return 0;
    }
    
    uint32_t absolute_pulses = (uint32_t)((position_mm / SCREW_LEAD_MM) * PULSE_PER_ROUND);
    Emm_V5_Pos_Control(&g_app_stepper, 0, speed_rpm, 15, absolute_pulses, true, false);
    
    return 1;
}

/**
  * @brief    紧急停止
  */
void Stepper_App_EmergencyStop(void)
{
    Emm_V5_Stop_Now(&g_app_stepper, false);
}

/**
  * @brief    DMA中断/解析接收帧的接口
  */
void Stepper_App_Parse(uint8_t *rx_buf, uint8_t rx_len)
{
    printf(">> Stepper Rx [%d]:", rx_len);
    for (uint8_t i = 0; i < rx_len; i++)
    {
        printf(" %02X", rx_buf[i]);
    }
    printf("\r\n");
    
    Emm_V5_Parse_Frame(&g_app_stepper, rx_buf, rx_len);
}

/**
  * @brief    获取当前丝杠高度位置 (单位: mm)
  */
float Stepper_App_GetCurrentPosition(void)
{
    float position = (g_app_stepper.real_pos / 360.0f) * SCREW_LEAD_MM;
    return position;
}

/**
  * @brief    异步触发读取当前位置
  */
void Stepper_App_TriggerPositionRead(void)
{
    Emm_V5_Read_Sys_Params(&g_app_stepper, S_CPOS);
}

/**
  * @brief    获取当前丝杠系统状态
  */
StepperSysState_t Stepper_App_GetSystemState(void)
{
    return g_system_state;
}
