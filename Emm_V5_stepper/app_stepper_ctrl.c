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

/**
  * @brief    应用层丝杆电机系统初始化
  */
void Stepper_App_Init(UART_HandleTypeDef *huart, uint8_t addr)
{
    /* 1. 初始化底层的 Emm_V5 驱动句柄 */
    Emm_V5_Init(&g_app_stepper, huart, addr);
    
    /* 2. 软件状态机置为未初始化 */
    g_system_state = STEPPER_STATE_UNINIT;
    
    /* 3. 延时等待电机控制板供电完成 */
    HAL_Delay(500);
    
    /* 4. 配置电机的无限位碰撞回零底层参数 (写入电机 EEPROM 永久存储)
     * 参数含义：
     *   - 模式2：多圈无限位碰撞回零 (堵转回零)
     *   - 方向1：CCW (逆时针方向，即滑块朝电机轴心一端回缩)
     *   - 回零速度：40 RPM (低速撞击，保护丝杆)
     *   - 回零超时：12000 ms (12秒超时退出)
     *   - 判定转速：15 RPM (当转速受阻降到15以下时)
     *   - 判定电流：350 mA (当堵转电流高于350mA时)
     *   - 判定时间：200 ms (以上卡死条件持续200ms则判定到位)
     *   - 上电不自启：false
     */
    Emm_V5_Origin_Modify_Params(&g_app_stepper, true, 2, EMM_CCW, 40, 12000, 15, 350, 200, false);
    
    /* 5. 稍微延时，确保电机配置保存完毕 */
    HAL_Delay(150);
}

/**
  * @brief    执行上电碰撞自动寻原点流程 (阻塞查询方式)
  */
uint8_t Stepper_App_ExecuteHoming(void)
{
    g_system_state = STEPPER_STATE_HOMING;
    
    /* 1. 确保电机已使能加电 */
    Emm_V5_En_Control(&g_app_stepper, true, false);
    HAL_Delay(100);
    
    /* 2. 发送指令触发碰撞回零寻原点 (模式2) */
    Emm_V5_Origin_Trigger_Return(&g_app_stepper, 2, false);
    
    /* 3. 循环等待并校验电机返回的回零状态 */
    uint32_t start_time = HAL_GetTick();
    uint8_t rx_buf[8] = {0};
    
    while (1)
    {
        HAL_Delay(250); /* 每隔250ms发送一次状态查询 */
        
        /* 强制清理可能残存的溢出错误，发送回零状态 S_ORG 查询指令 */
        __HAL_UART_CLEAR_OREFLAG(g_app_stepper.huart);
        HAL_UART_AbortReceive(g_app_stepper.huart);
        
        Emm_V5_Read_Sys_Params(&g_app_stepper, S_ORG);
        
        /* 阻塞等待 4 字节的返回包：[地址] [0x3B] [回零状态值] [0x6B] */
        if (HAL_UART_Receive(g_app_stepper.huart, rx_buf, 4, 100) == HAL_OK)
        {
            if (rx_buf[0] == g_app_stepper.addr && rx_buf[1] == 0x3B)
            {
                g_app_stepper.origin_state = rx_buf[2]; /* 0: 回零中/未开始, 1: 成功, 2: 失败 */
            }
        }
        
        /* 回零成功处理 */
        if (g_app_stepper.origin_state == 1)
        {
            /* A. 成功撞墙，将此当前物理卡死位置标记为软件坐标绝对 0 点 (零度) */
            Emm_V5_Reset_CurPos_To_Zero(&g_app_stepper);
            HAL_Delay(100);
            
            /* B. 为了防止滑块死死咬在物理挡板上导致后续运动卡死，
             *    必须立刻朝反方向（CW）前进一段安全避让距离（例如 4mm）
             *    4mm 对应的脉冲换算：(4.0f / 8.0f) * 3200 = 1600 脉冲
             */
            uint32_t back_pulses = (uint32_t)((SAFE_CLEARANCE_MM / SCREW_LEAD_MM) * PULSE_PER_ROUND);
            Emm_V5_Pos_Control(&g_app_stepper, EMM_CW, 500, 10, back_pulses, false, false);
            
            /* 等待反向避让动作执行完毕 */
            HAL_Delay(800); 
            
            /* C. 再次将此安全位置强制复位清零作为后续业务运动的起点 0 坐标点 */
            Emm_V5_Reset_CurPos_To_Zero(&g_app_stepper);
            HAL_Delay(100);
            
            g_system_state = STEPPER_STATE_READY;
            return 1;
        }
        
        /* 回零失败处理 */
        if (g_app_stepper.origin_state == 2)
        {
            g_system_state = STEPPER_STATE_ERROR;
            return 0;
        }
        
        /* 超过15秒未撞墙判定超时保护，防止顶死电机 */
        if (HAL_GetTick() - start_time > 15000)
        {
            Emm_V5_Origin_Interrupt(&g_app_stepper); /* 强制中断回零 */
            g_system_state = STEPPER_STATE_ERROR;
            return 0;
        }
    }
}

/**
  * @brief    控制滑块移动到丝杆行程的绝对物理位置 (带软限位保护)
  */
uint8_t Stepper_App_MoveToPosition(float position_mm, uint16_t speed_rpm)
{
    /* 1. 安全保护：必须先回零校准基准才允许正常运动 */
    if (g_system_state != STEPPER_STATE_READY)
    {
        return 0;
    }
    
    /* 2. 软件行程范围限位保护，防止滑块撞墙损坏丝杆 */
    if (position_mm < 0.0f || position_mm > SCREW_MAX_TRAVEL_MM)
    {
        return 0; /* 拒绝越界执行 */
    }
    
    /* 3. 物理距离(mm)换算为软件的绝对脉冲数
     * 公式: (目标距离 / 丝杆导程) * 单圈脉冲数
     */
    uint32_t absolute_pulses = (uint32_t)((position_mm / SCREW_LEAD_MM) * PULSE_PER_ROUND);
    
    /* 4. 调用绝对位置控制接口 (raF = true)
     * 注意：绝对位置控制模式下，dir 参数在发送后驱动板会自动根据当前所处位置决定旋转方向，
     *       因此这里 dir 参数固定填 0 即可。
     *       加速度参数设为 15 (S型平滑减速，防止瞬间停机丢步)。
     */
    Emm_V5_Pos_Control(&g_app_stepper, 0, speed_rpm, 15, absolute_pulses, true, false);
    
    return 1;
}

/**
  * @brief    紧急停止电机的运动
  */
void Stepper_App_EmergencyStop(void)
{
    Emm_V5_Stop_Now(&g_app_stepper, false);
}

/**
  * @brief    获取当前电机的绝对物理坐标 (单位: mm)
  */
float Stepper_App_GetCurrentPosition(void)
{
    /* 阻塞式读取当前绝对角度，成功则计算并转换为毫米 */
    if (Emm_V5_Read_Position_Blocking(&g_app_stepper))
    {
        /* 角度转换为毫米公式: (当前绝对角度 / 360.0f) * 丝杆导程 */
        float position = (g_app_stepper.real_pos / 360.0f) * SCREW_LEAD_MM;
        return position;
    }
    
    return -1.0f; /* 读取出错返回-1 */
}

/**
  * @brief    获取当前丝杆系统所处的运行状态
  */
StepperSysState_t Stepper_App_GetSystemState(void)
{
    return g_system_state;
}
