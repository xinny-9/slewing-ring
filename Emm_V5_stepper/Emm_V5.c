/**
 * *****************************************************************************
 * @file    Emm_V5.c
 * @brief   Emm_V5.0 闭环步进电机驱动库实现 (基于 STM32 HAL 库)
 * *****************************************************************************
 */

#include "Emm_V5.h"
#include <string.h>

/**
  * @brief    发送指令的底层物理实现 (调用 HAL_UART_Transmit)
  */
static void Emm_V5_SendCmd(Emm_V5_Motor *motor, uint8_t *cmd, uint8_t len)
{
    if (motor != NULL && motor->huart != NULL)
    {
        HAL_UART_Transmit(motor->huart, cmd, len, 100);
    }
}

/**
  * @brief    初始化电机句柄
  */
void Emm_V5_Init(Emm_V5_Motor *motor, UART_HandleTypeDef *huart, uint8_t addr)
{
    if (motor != NULL)
    {
        motor->addr = addr;
        motor->huart = huart;
        motor->en_state = 0;
        motor->arrive_state = 0;
        motor->lck_state = 0;
        motor->origin_state = 0;
        motor->real_pos = 0.0f;
        motor->real_vel = 0.0f;
        motor->err_pos = 0.0f;
        motor->target_pos = 0.0f;
    }
}

/**
  * @brief    电机使能控制
  * @param    state : 使使能状态，true为使能电机，false为关闭电机
  * @param    snF   : 多机同步标志，false为不启用，true为启用
  */
void Emm_V5_En_Control(Emm_V5_Motor *motor, bool state, bool snF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;                 /* 地址 */
    cmd[1] = 0xF3;                        /* 功能码 */
    cmd[2] = 0xAB;                        /* 辅助码 */
    cmd[3] = (uint8_t)state;              /* 使能状态：1-使能，0-失能 */
    cmd[4] = snF ? 1 : 0;                 /* 多机同步标志 */
    cmd[5] = 0x6B;                        /* 校验字节 */
    
    Emm_V5_SendCmd(motor, cmd, 6);
}

/**
  * @brief    速度模式控制
  * @param    dir   : 方向，0为CW，其余值为CCW
  * @param    vel   : 速度，范围 0 - 5000 RPM
  * @param    acc   : 加速度，范围 0 - 255 (0为直接启动)
  * @param    snF   : 多机同步标志，false为不启用，true为启用
  */
void Emm_V5_Vel_Control(Emm_V5_Motor *motor, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = motor->addr;                 /* 地址 */
    cmd[1] = 0xF6;                        /* 功能码 */
    cmd[2] = dir;                         /* 方向 */
    cmd[3] = (uint8_t)(vel >> 8);         /* 速度高8位 */
    cmd[4] = (uint8_t)(vel >> 0);         /* 速度低8位 */
    cmd[5] = acc;                         /* 加速度 */
    cmd[6] = snF ? 1 : 0;                 /* 多机同步标志 */
    cmd[7] = 0x6B;                        /* 校验字节 */
    
    Emm_V5_SendCmd(motor, cmd, 8);
}

/**
  * @brief    位置模式控制
  * @param    dir   : 方向，0为CW，其余值为CCW
  * @param    vel   : 速度(RPM)
  * @param    acc   : 加速度
  * @param    clk   : 脉冲数
  * @param    raF   : 相对/绝对标志，false为相对，true为绝对
  * @param    snF   : 多机同步标志
  */
void Emm_V5_Pos_Control(Emm_V5_Motor *motor, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    cmd[0]  = motor->addr;                /* 地址 */
    cmd[1]  = 0xFD;                       /* 功能码 */
    cmd[2]  = dir;                        /* 方向 */
    cmd[3]  = (uint8_t)(vel >> 8);        /* 速度高8位 */
    cmd[4]  = (uint8_t)(vel >> 0);        /* 速度低8位 */
    cmd[5]  = acc;                        /* 加速度 */
    cmd[6]  = (uint8_t)(clk >> 24);       /* 脉冲数 bit24-31 */
    cmd[7]  = (uint8_t)(clk >> 16);       /* 脉冲数 bit16-23 */
    cmd[8]  = (uint8_t)(clk >> 8);        /* 脉冲数 bit8-15 */
    cmd[9]  = (uint8_t)(clk >> 0);        /* 脉冲数 bit0-7 */
    cmd[10] = raF ? 1 : 0;                /* 相对/绝对标志 */
    cmd[11] = snF ? 1 : 0;                /* 多机同步标志 */
    cmd[12] = 0x6B;                       /* 校验字节 */
    
    Emm_V5_SendCmd(motor, cmd, 13);
}

/**
  * @brief    让电机立即停止运动
  */
void Emm_V5_Stop_Now(Emm_V5_Motor *motor, bool snF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF ? 1 : 0;
    cmd[4] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 5);
}

/**
  * @brief    触发多机同步开始运动
  */
void Emm_V5_Synchronous_motion(Emm_V5_Motor *motor)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0xFF;
    cmd[2] = 0x66;
    cmd[3] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 4);
}

/**
  * @brief    将当前位置清零
  */
void Emm_V5_Reset_CurPos_To_Zero(Emm_V5_Motor *motor)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0x0A;
    cmd[2] = 0x6D;
    cmd[3] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 4);
}

/**
  * @brief    解除堵转保护
  */
void Emm_V5_Reset_Clog_Pro(Emm_V5_Motor *motor)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0x0E;
    cmd[2] = 0x52;
    cmd[3] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 4);
}

/**
  * @brief    修改开环/闭环控制模式
  */
void Emm_V5_Modify_Ctrl_Mode(Emm_V5_Motor *motor, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0x46;
    cmd[2] = 0x69;
    cmd[3] = svF ? 1 : 0;
    cmd[4] = ctrl_mode;
    cmd[5] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 6);
}

/**
  * @brief    设置单圈回零的零点位置
  */
void Emm_V5_Origin_Set_O(Emm_V5_Motor *motor, bool svF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0x93;
    cmd[2] = 0x88;
    cmd[3] = svF ? 1 : 0;
    cmd[4] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 5);
}

/**
  * @brief    修改回零参数
  */
void Emm_V5_Origin_Modify_Params(Emm_V5_Motor *motor, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0x4C;
    cmd[2] = 0xAE;
    cmd[3] = svF ? 1 : 0;
    cmd[4] = o_mode;
    cmd[5] = o_dir;
    cmd[6] = (uint8_t)(o_vel >> 8);
    cmd[7] = (uint8_t)(o_vel >> 0);
    cmd[8] = (uint8_t)(o_tm >> 24);
    cmd[9] = (uint8_t)(o_tm >> 16);
    cmd[10] = (uint8_t)(o_tm >> 8);
    cmd[11] = (uint8_t)(o_tm >> 0);
    cmd[12] = (uint8_t)(sl_vel >> 8);
    cmd[13] = (uint8_t)(sl_vel >> 0);
    cmd[14] = (uint8_t)(sl_ma >> 8);
    cmd[15] = (uint8_t)(sl_ma >> 0);
    cmd[16] = (uint8_t)(sl_ms >> 8);
    cmd[17] = (uint8_t)(sl_ms >> 0);
    cmd[18] = potF ? 1 : 0;
    cmd[19] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 20);
}

/**
  * @brief    发送命令触发回零
  */
void Emm_V5_Origin_Trigger_Return(Emm_V5_Motor *motor, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0x9A;
    cmd[2] = o_mode;
    cmd[3] = snF ? 1 : 0;
    cmd[4] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  */
void Emm_V5_Origin_Interrupt(Emm_V5_Motor *motor)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = motor->addr;
    cmd[1] = 0x9C;
    cmd[2] = 0x48;
    cmd[3] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, 4);
}

/**
  * @brief    向电机发送读取系统参数的指令 (非阻塞)
  */
void Emm_V5_Read_Sys_Params(Emm_V5_Motor *motor, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};
    
    cmd[i++] = motor->addr;

    switch(s)
    {
        case S_VER  : cmd[i++] = 0x1F; break;
        case S_RL   : cmd[i++] = 0x20; break;
        case S_PID  : cmd[i++] = 0x21; break;
        case S_VBUS : cmd[i++] = 0x24; break;
        case S_CPHA : cmd[i++] = 0x27; break;
        case S_ENCL : cmd[i++] = 0x31; break;
        case S_TPOS : cmd[i++] = 0x33; break;
        case S_VEL  : cmd[i++] = 0x35; break;
        case S_CPOS : cmd[i++] = 0x36; break;
        case S_PERR : cmd[i++] = 0x37; break;
        case S_FLAG : cmd[i++] = 0x3A; break;
        case S_ORG  : cmd[i++] = 0x3B; break;
        case S_Conf : cmd[i++] = 0x42; cmd[i++] = 0x6C; break;
        case S_State: cmd[i++] = 0x43; cmd[i++] = 0x7A; break;
        default: return;
    }

    cmd[i++] = 0x6B;
    
    Emm_V5_SendCmd(motor, cmd, i);
}

/**
  * @brief    阻塞式读取电机当前实时角度 (发送指令并同步等待接收，更新 real_pos)
  */
bool Emm_V5_Read_Position_Blocking(Emm_V5_Motor *motor)
{
    uint8_t rx_buf[8] = {0};
    
    if (motor == NULL || motor->huart == NULL) return false;
    
    /* 清理可能残留的接收缓冲 */
    __HAL_UART_CLEAR_OREFLAG(motor->huart);
    HAL_UART_AbortReceive(motor->huart);
    
    /* 发送读取实时位置指令 */
    Emm_V5_Read_Sys_Params(motor, S_CPOS);
    
    /* 阻塞式等待接收8个字节的返回数据 */
    if (HAL_UART_Receive(motor->huart, rx_buf, 8, EMM_BLOCKING_TIMEOUT) == HAL_OK)
    {
        /* 校验返回数据的帧头部 */
        if (rx_buf[0] == motor->addr && rx_buf[1] == 0x36)
        {
            uint32_t pos_val = (uint32_t)(
                                ((uint32_t)rx_buf[3] << 24) |
                                ((uint32_t)rx_buf[4] << 16) |
                                ((uint32_t)rx_buf[5] << 8)  |
                                ((uint32_t)rx_buf[6] << 0)
                               );
            float angle = (float)pos_val * 360.0f / 65536.0f;
            if (rx_buf[2]) { angle = -angle; }
            motor->real_pos = angle;
            return true;
        }
    }
    
    return false;
}

/**
  * @brief    阻塞式读取电机当前实时转速 (发送指令并同步等待接收，更新 real_vel)
  */
bool Emm_V5_Read_Speed_Blocking(Emm_V5_Motor *motor)
{
    uint8_t rx_buf[8] = {0};
    
    if (motor == NULL || motor->huart == NULL) return false;
    
    /* 清理接收缓冲 */
    __HAL_UART_CLEAR_OREFLAG(motor->huart);
    HAL_UART_AbortReceive(motor->huart);
    
    /* 发送读取转速指令 */
    Emm_V5_Read_Sys_Params(motor, S_VEL);
    
    /* 阻塞式等待接收6个字节的返回数据 */
    if (HAL_UART_Receive(motor->huart, rx_buf, 6, EMM_BLOCKING_TIMEOUT) == HAL_OK)
    {
        /* 校验返回数据帧头 */
        if (rx_buf[0] == motor->addr && rx_buf[1] == 0x35)
        {
            uint16_t vel_val = (uint16_t)(
                                ((uint16_t)rx_buf[3] << 8) |
                                ((uint16_t)rx_buf[4] << 0)
                               );
            float speed = (float)vel_val;
            if (rx_buf[2]) { speed = -speed; }
            motor->real_vel = speed;
            return true;
        }
    }
    
    return false;
}

/**
  * @brief    阻塞式读取电机当前使能/到位/堵转状态
  */
bool Emm_V5_Read_Status_Blocking(Emm_V5_Motor *motor)
{
    uint8_t rx_buf[8] = {0};
    
    if (motor == NULL || motor->huart == NULL) return false;
    
    /* 清理接收缓冲 */
    __HAL_UART_CLEAR_OREFLAG(motor->huart);
    HAL_UART_AbortReceive(motor->huart);
    
    /* 发送读取使能/到位/堵转标志指令 */
    Emm_V5_Read_Sys_Params(motor, S_FLAG);
    
    /* 阻塞式等待接收4个字节的返回数据 */
    if (HAL_UART_Receive(motor->huart, rx_buf, 4, EMM_BLOCKING_TIMEOUT) == HAL_OK)
    {
        /* 校验返回数据帧头 */
        if (rx_buf[0] == motor->addr && rx_buf[1] == 0x3A)
        {
            motor->en_state     = (rx_buf[2] & 0x01) ? 1 : 0;
            motor->arrive_state = (rx_buf[2] & 0x02) ? 1 : 0;
            motor->lck_state    = (rx_buf[2] & 0x04) ? 1 : 0;
            return true;
        }
    }
    
    return false;
}

/**
  * @brief    统一的帧解析接口 (非阻塞，适用于空闲中断/DMA断帧接收后的数据处理)
  */
bool Emm_V5_Parse_Frame(Emm_V5_Motor *motor, uint8_t *rx_buf, uint8_t rx_len)
{
    if (motor == NULL || rx_buf == NULL || rx_len < 4) return false;
    
    /* 检查是否是本电机回复的数据 */
    if (rx_buf[0] != motor->addr) return false;
    
    /* 校验末尾的固定校验码 */
    if (rx_buf[rx_len - 1] != 0x6B) return false;
    
    /* 根据功能码解析不同类型的回复 */
    switch (rx_buf[1])
    {
        case 0x36: /* 实时位置回复帧 (长度通常为 8 字节) */
            if (rx_len >= 8)
            {
                uint32_t pos_val = (uint32_t)(
                                    ((uint32_t)rx_buf[3] << 24) |
                                    ((uint32_t)rx_buf[4] << 16) |
                                    ((uint32_t)rx_buf[5] << 8)  |
                                    ((uint32_t)rx_buf[6] << 0)
                                   );
                float angle = (float)pos_val * 360.0f / 65536.0f;
                if (rx_buf[2]) { angle = -angle; }
                motor->real_pos = angle;
                return true;
            }
            break;
            
        case 0x35: /* 实时速度回复帧 (长度通常为 6 字节) */
            if (rx_len >= 6)
            {
                uint16_t vel_val = (uint16_t)(
                                    ((uint16_t)rx_buf[3] << 8) |
                                    ((uint16_t)rx_buf[4] << 0)
                                   );
                float speed = (float)vel_val;
                if (rx_buf[2]) { speed = -speed; }
                motor->real_vel = speed;
                return true;
            }
            break;
            
        case 0x3A: /* 标志位状态回复帧 (长度通常为 4 字节) */
            if (rx_len >= 4)
            {
                motor->en_state     = (rx_buf[2] & 0x01) ? 1 : 0;
                motor->arrive_state = (rx_buf[2] & 0x02) ? 1 : 0;
                motor->lck_state    = (rx_buf[2] & 0x04) ? 1 : 0;
                return true;
            }
            break;
            
        case 0x3B: /* 回零状态回复帧 (长度通常为 4 字节) */
            if (rx_len >= 4)
            {
                motor->origin_state = rx_buf[2]; /* 0: 回零中/未开始, 1: 成功, 2: 失败 */
                return true;
            }
            break;
            
        case 0x33: /* 目标位置回复帧 (长度为 8 字节) */
            if (rx_len >= 8)
            {
                uint32_t pos_val = (uint32_t)(
                                    ((uint32_t)rx_buf[3] << 24) |
                                    ((uint32_t)rx_buf[4] << 16) |
                                    ((uint32_t)rx_buf[5] << 8)  |
                                    ((uint32_t)rx_buf[6] << 0)
                                   );
                float angle = (float)pos_val * 360.0f / 65536.0f;
                if (rx_buf[2]) { angle = -angle; }
                motor->target_pos = angle;
                return true;
            }
            break;
            
        case 0x37: /* 位置误差回复帧 (长度为 8 字节) */
            if (rx_len >= 8)
            {
                uint32_t pos_val = (uint32_t)(
                                    ((uint32_t)rx_buf[3] << 24) |
                                    ((uint32_t)rx_buf[4] << 16) |
                                    ((uint32_t)rx_buf[5] << 8)  |
                                    ((uint32_t)rx_buf[6] << 0)
                                   );
                float angle = (float)pos_val * 360.0f / 65536.0f;
                if (rx_buf[2]) { angle = -angle; }
                motor->err_pos = angle;
                return true;
            }
            break;
            
        default:
            break;
    }
    
    return false;
}
