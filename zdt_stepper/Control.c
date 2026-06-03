#include "CONTROL.h"
#include "Data.h"

/**
 * @file    Control.c
 * @brief   步进电机控制模块实现文件
 * @details 提供电机使能、速度控制、位置控制、细分设置和地址修改等功能
 *          通过UART串口与电机驱动器通信
 */

// CRC8校验表（已注释，当前使用固定校验值0x6B）
//static const uint8_t crc8Table[256] = {
//0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, // 0
//0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41, // 8
//0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, // 16
//0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC, // 24
//0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, // 32
//0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62, // 40
//0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, // 48
//0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF, // 56
//0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, // 64
//0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07, // 72
//0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, // 80
//0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A, // 88
//0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, // 96
//0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24, // 104
//0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, // 112
//0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9, // 120
//0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, // 128
//0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD, // 136
//0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, // 144
//0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50, // 152
//0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, // 160
//0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE, // 168
//0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, // 176
//0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73, // 184
//0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, // 192
//0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B, // 200
//0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, // 208
//0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16, // 216
//0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, // 224
//0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8, // 232
//0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, // 240
//0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35 // 248
//};

// 函数声明
void Motor_Position(Motor_Control *Motor,uint8_t Direction,uint16_t Speed,uint8_t Accelerate,uint32_t Where);
//uint8_t cmd_calCRC8(uint8_t *p, uint8_t len);

/**
 * @file    Control.c
 * @brief   步进电机控制模块 - 初始化函数
 * @details 该函数用于初始化电机控制结构体，为后续的电机控制操作做准备
 * @param   Motor  电机控制结构体指针，包含电机地址和UART句柄
 * @param   Motor_Address  电机地址，用于在UART通信中标识电机
 */
void Control_Init(Motor_Control *Motor,UART_HandleTypeDef *huart,uint8_t Motor_Address)
{
    Motor->Motor_Address = Motor_Address;      
    Motor->Usart = huart;         
    Motor->State =0;
    Motor->EN_State=0;
    Motor->LOCK_State=0;
    Motor->POINT_State=0;
    Motor->Motor_Blind_Time = HAL_GetTick() + 500;
}

/**
 * @brief   电机使能控制函数
 * @param   Motrol  电机控制结构体指针，包含电机地址和UART句柄
 * @param   ENABLE  使能状态：1-使能电机，0-失能电机
 * @details 发送使能命令到电机驱动器，命令格式：[地址][0xF3][0xAB][使能状态][0x00][校验码]
 *          先调用Control_Position(0,0,0,0)复位电机状态
 */
void Control_En(Motor_Control *Motor,uint8_t ENABLE)
{
    Motor_Position(Motor,0,0,0,0);
    uint8_t Tx_Data[6]={0};
    Tx_Data[0]=Motor->Motor_Address;    // 电机地址
    Tx_Data[1]=0xF3;                      // 使能命令码
    Tx_Data[2]=0xAB;                      // 命令子码
    Tx_Data[3]=ENABLE;                    // 使能状态：1使能，0失能
    Tx_Data[4]=0x00;                      // 保留字节
    Tx_Data[5]=0x6B;                      // 固定校验值（可替换为CRC8计算）
    //Tx_Data[5]=cmd_calCRC8(Tx_Data,5);
    
    HAL_UART_Transmit(Motor->Usart,Tx_Data,6,100);  // 通过UART发送6字节数据，超时100ms
}

/**
 * @brief   电机速度控制函数
 * @param   Motrol      电机控制结构体指针
 * @param   Direction   旋转方向：0-顺时针，1-逆时针
 * @param   Speed       速度值（0-3000），超过3000会被限制为3000
 * @param   Accelerate  加速度值
 * @details 发送速度控制命令，命令格式：[地址][0xF6][方向][速度高字节][速度低字节][加速度][0x00][校验码]
 */
void Control_Speed(Motor_Control *Motor,uint8_t Direction,uint16_t Speed,uint8_t Accelerate)
{
    uint8_t Tx_Data[8]={0};
    Tx_Data[0]=Motor->Motor_Address;  // 电机地址
    Tx_Data[1]=0xF6;                      // 速度控制命令码
    Tx_Data[2]=Direction;                 // 旋转方向
    
    if (Speed > 3000) 
        {
    Speed = 3000;  // 速度上限限制为3000
    }
        
    uint8_t Speed1 = (Speed >> 8) & 0xFF;  // 速度高8位
    uint8_t Speed2 = Speed & 0xFF;         // 速度低8位    
    
    Tx_Data[3]=Speed1;                     // 速度高字节
    Tx_Data[4]=Speed2;                     // 速度低字节
    Tx_Data[5]=Accelerate&0xFF;            // 加速度
    Tx_Data[6]=0x00;                       // 保留字节
    Tx_Data[7]=0x6B;                       // 固定校验值
    //Tx_Data[7]=cmd_calCRC8(Tx_Data,7);
    
    HAL_UART_Transmit(Motor->Usart,Tx_Data,8,100);  // 发送8字节数据
}

/**
 * @brief   电机位置控制函数
 * @param   Motor      电机控制结构体指针
 * @param   Direction   旋转方向
 * @param   Speed       运行速度（0-3000）
 * @param   Accelerate  加速度值
 * @param   Where       目标位置，支持预定义位置：Location_1_4、Location_1_2、Location_3_4、Location_1
 * @details 发送位置控制命令，根据目标位置更新电机状态
 *          命令格式：[地址][0xFD][方向][速度高][速度低][加速度][位置3][位置2][位置1][位置0][0x02][0x00][校验码]
 */
void Motor_Position(Motor_Control *Motor,uint8_t Direction,uint16_t Speed,uint8_t Accelerate,uint32_t Where)
{   
    uint8_t Tx_Data[13]={0};
    switch(Where)
    {
        case 0:
        Motor->State=0;  // 复位状态
        break;
        case Location_1_4:
        Motor->State=1;  // 1/4位置状态
        break;
        case Location_1_2:
        Motor->State=2;  // 1/2位置状态
        break;
        case Location_3_4:
        Motor->State=3;  // 3/4位置状态
        break;
        case Location_1:
        Motor->State=4;  // 1圈位置状态
        break;
        default:
        Motor->State=0;  // 默认复位状态
        break;
    }
    
    Tx_Data[0]=Motor->Motor_Address;  // 电机地址
    Tx_Data[1]=0xFD;                      // 位置控制命令码
    Tx_Data[2]=Direction;                 // 旋转方向
    
    if (Speed > 3000) 
        {
    Speed = 3000;  // 速度限制
    }
        
    uint8_t Speed1 = (Speed >> 8) & 0xFF;  // 速度高字节
    uint8_t Speed2 = Speed & 0xFF;         // 速度低字节        
    
    Tx_Data[3]=Speed1;
    Tx_Data[4]=Speed2;
    Tx_Data[5]=Accelerate&0xFF;
    
    Tx_Data[6]=(Where>>24)&0xFF;  // 位置最高字节
    Tx_Data[7]=(Where>>16)&0xFF;  // 位置次高字节
    Tx_Data[8]=(Where>>8) &0xFF;  // 位置次低字节
    Tx_Data[9]= Where     &0xFF;  // 位置最低字节
    
    Tx_Data[10]=0x02;  // 模式
    Tx_Data[11]=0x00;  // 保留字节
    Tx_Data[12]=0x6B;  // 固定校验值
    //Tx_Data[12]=cmd_calCRC8(Tx_Data,12);

    HAL_UART_Transmit(Motor->Usart,Tx_Data,13,100);  // 发送13字节数据
}

/**
 * @brief   电机细分设置函数
 * @param   Motrol      电机控制结构体指针
 * @param   Subdivide   细分值，决定步进电机的每转步数
 * @details 设置电机驱动器的细分参数，影响电机精度和平滑度
 *          命令格式：[地址][0x84][0x8A][0x01][细分值][校验码]
 */
void Control_Subdivide(Motor_Control *Motor,uint8_t Subdivide)
{
    uint8_t Tx_Data[6]={0}; 
    Tx_Data[0]=Motor->Motor_Address;  // 电机地址
    Tx_Data[1]=0x84;                      // 细分设置命令码
    Tx_Data[2]=0x8A;                      // 命令子码
    Tx_Data[3]=0x01;                      // 参数标识
    Tx_Data[4]=Subdivide&0xFF;            // 细分值
    Tx_Data[5]=0x6B;                      // 固定校验值
    //Tx_Data[5]=cmd_calCRC8(Tx_Data,5);
    
    HAL_UART_Transmit(Motor->Usart,Tx_Data,6,100);  // 发送6字节数据
}

/**
 * @brief   电机地址修改函数
 * @param   Motrol  电机控制结构体指针
 * @param   Address 新地址值（1-255），地址0会被自动改为1
 * @details 修改电机驱动器的通信地址，用于多电机系统中的地址管理
 *          命令格式：[当前地址][0xAE][0x4B][0x01][新地址][校验码]
 */
void Control_Address(Motor_Control *Motor,uint8_t Address)
{
    uint8_t Tx_Data[6]={0};  
    
    if(Address==0){Address=1;}  // 地址不能为0，最小为1
    
    Tx_Data[0]=Motor->Motor_Address;  // 当前电机地址
    Tx_Data[1]=0xAE;                      // 地址修改命令码
    Tx_Data[2]=0x4B;                      // 命令子码
    Tx_Data[3]=0x01;                      // 参数标识
    Tx_Data[4]=Address&0xFF;              // 新地址
    Tx_Data[5]=0x6B;                      // 固定校验值
    //Tx_Data[5]=cmd_calCRC8(Tx_Data,5);

    HAL_UART_Transmit(Motor->Usart,Tx_Data,6,100);  // 发送6字节数据
}

/**
 * @file    Control.c
 * @brief   步进电机控制模块 - 数据周期性读取函数
 * @details 该函数用于周期性地从电机驱动器读取状态数据，实现电机状态的实时监控
 *          数据读取频率由Reading_Time参数指定，单位为毫秒
 *         通过调用Zhang_Reading_Data函数获取电机状态数据，并更新Motor结构体中的时间戳
 * @param   Motor  电机控制结构体指针，包含电机地址和UART句柄
 * @param   Reading_Time  数据读取时间间隔，单位为毫秒 
 */
void Motor_Reading(Motor_Control *Motor,uint32_t Reading_Time)
{
    Motor->Current_Time=HAL_GetTick();

    if(Motor->Current_Time -Motor->Last_Time >= Reading_Time)
    {
     Motor->Last_Time=Motor->Current_Time;
     Zhang_Reading_Data(Motor);
    }
}

/**
 * @brief   电机定位控制函数（带盲区时间保护）
 * @param   Motor      电机控制结构体指针
 * @param   Speed      运行速度（0-3000）
 * @param   Accelerate 加速度值
 * @param   Where      目标位置
 * @details 实现带时间保护的电机定位控制功能
 *          工作流程：
 *          - 检查当前时间是否超过盲区时间（Motor_Blind_Time）
 *          - 检查定位状态标志（POINT_State）是否为1
 *          - 如果条件满足，执行定位操作：
 *            * 清除定位状态标志
 *            * 调用 Motor_Position 函数执行位置控制（方向设为0）
 *            * 设置新的盲区时间为当前时间+500ms，防止频繁触发
 * @note    该函数通过盲区时间机制避免电机定位操作过于频繁
 */
uint8_t Motor_Want_Position(Motor_Control *Motor,uint16_t Speed,uint8_t Accelerate,uint32_t Where)
{
    if(Motor->Current_Time >= Motor->Motor_Blind_Time)
    {
        if(Motor->POINT_State==1)
        {
        Motor->POINT_State=0;
        Motor_Position(Motor,0,Speed,Accelerate,Where);
        Motor->Motor_Blind_Time = Motor->Current_Time + 500;
            return 1;
        }
    }
    return 0;
}

/**
 * @brief   CRC8校验计算函数（已注释）
 * @param   p   数据指针
 * @param   len 数据长度
 * @return  CRC8校验值
 * @details 使用查表法计算CRC8校验码，当前代码使用固定值0x6B代替
 *          如需启用CRC校验，取消注释此函数并修改各函数中的校验码赋值
 */

//uint8_t cmd_calCRC8(uint8_t *p, uint8_t len)
//{
//    unsigned char i = 0, crc8 = 0x00;   
//    for(i = 0; i < len; i++) {        
//        crc8 = crc8Table[crc8 ^ p[i]];  // 查表计算CRC8
//    }
//    return crc8;
//}
