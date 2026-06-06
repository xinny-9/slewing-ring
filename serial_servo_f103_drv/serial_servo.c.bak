/**
 * *****************************************************************************
 * @file    serial_servo.c
 * @author  Antigravity
 * @brief   串口总线舵机核心协议接口实现文件 (纯协议层，与底层硬件完全解耦)
 * *****************************************************************************
 */

#include "serial_servo.h"
#include <string.h>

#define GET_LOW_BYTE(A)  ((uint8_t)(A))           
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))     
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B)) 

/**
 * ============================================================================
 * @brief   命令帧初始化函数(静态)
 * @details 初始化一个串口舵机通信协议的命令帧结构体，设置帧头和基本信息
 * @param   frame    - 指向命令帧结构体的指针
 * @param   servo_id - 目标舵机的ID
 * @param   cmd      - 要发送的命令字节
 * @return  无
 * @note    这是一个内部辅助函数，用于所有命令发送前的帧格式初始化
 * ============================================================================
 */
static void cmd_frame_init(SerialServoCmdTypeDef *frame, uint8_t servo_id, uint8_t cmd)
{
    frame->header_1 = SERIAL_SERVO_FRAME_HEADER;  // 设置第一个帧头
    frame->header_2 = SERIAL_SERVO_FRAME_HEADER;  // 设置第二个帧头
    frame->elements.servo_id = servo_id;          // 设置目标舵机ID
    frame->elements.command = cmd;                // 设置命令字节
}

/**
 * ============================================================================
 * @brief   命令帧完成函数(静态)
 * @details 计算并设置命令帧的数据长度和校验和，使帧成为可发送状态
 * @param   frame    - 指向命令帧结构体的指针
 * @param   args_num - 本条命令包含的参数个数
 * @return  无
 * @note    必须在所有参数填充完毕后调用此函数，以完善帧的长度和校验字段
 * ============================================================================
 */
static void cmd_frame_complete(SerialServoCmdTypeDef *frame, uint8_t args_num)
{
    frame->elements.length = args_num + 3;  // 长度 = 参数数 + 舵机ID + 命令 + 长度字段
    frame->elements.args[args_num] = serial_servo_checksum((uint8_t*)frame);  // 计算并填入校验和
}

/**
 * ============================================================================
 * @brief   设置舵机ID
 * @details 改变指定舵机的ID号，需要提供旧ID以定位该舵机
 * @param   self   - 指向舵机控制器的指针
 * @param   old_id - 舵机的当前ID
 * @param   new_id - 要设置的新ID
 * @return  无
 * @note    设置后需要重启舵机才能生效；新ID范围应为0-253
 * ============================================================================
 */
void serial_servo_set_id(SerialServoControllerTypeDef *self, uint32_t old_id, uint32_t new_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)old_id, SERIAL_SERVO_ID_WRITE);  // 使用旧ID来寻址
    frame.elements.args[0] = (uint8_t)new_id;                         // 设置新ID
    cmd_frame_complete(&frame, 1);                                   // 完成帧，1个参数
    self->serial_write_and_read(self, &frame, true);                 // 发送命令（只写，不需要读响应）
}

/**
 * ============================================================================
 * @brief   读取舵机ID
 * @details 查询指定舵机当前的ID号，用于验证或确认舵机标识
 * @param   self        - 指向舵机控制器的指针
 * @param   servo_id    - 目标舵机的ID
 * @param   ret_servo_id - 指向返回结果的指针，存储读回的ID
 * @return  0表示成功读取，-1表示通信失败
 * @note    读取失败可能原因：通信超时、舵机不存在或无响应
 * ============================================================================
 */
int serial_servo_read_id(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *ret_servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ID_READ);  // 初始化读ID命令
    cmd_frame_complete(&frame, 0);                                   // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {     // 发送并等待响应
        *ret_servo_id = self->rx_frame.elements.args[0];             // 提取响应中的ID
        return 0;                                                     // 返回成功
    }
    return -1;                                                        // 返回失败
}

/**
 * ============================================================================
 * @brief   设置舵机位置和运动时间
 * @details 命令舵机在指定时间内移动到目标位置，实现平滑的舵机运动
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   position - 目标位置，范围0-1000，对应舵机的全行程
 * @param   duration - 运动时间(毫秒)，决定舵机移动速度
 * @return  无
 * @note    position会自动限制在0-1000范围内；duration的有效范围通常为0-5000ms
 * ============================================================================
 */
void serial_servo_set_position(SerialServoControllerTypeDef *self, uint32_t servo_id, int position, uint32_t duration)
{
    SerialServoCmdTypeDef frame;
    // 限制position在有效范围[0, 1000]
    if (position > 1000) position = 1000;  // 上限限制
    if (position < 0)    position = 0;     // 下限限制
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_MOVE_TIME_WRITE);  // 初始化设置位置命令
    frame.elements.args[0] = GET_LOW_BYTE(position);    // 位置低byte
    frame.elements.args[1] = GET_HIGH_BYTE(position);   // 位置高byte
    frame.elements.args[2] = GET_LOW_BYTE(duration);    // 时间低byte
    frame.elements.args[3] = GET_HIGH_BYTE(duration);   // 时间高byte
    cmd_frame_complete(&frame, 4);                      // 完成帧，4个参数
    self->serial_write_and_read(self, &frame, true);    // 发送命令（只写）
}

/**
 * ============================================================================
 * @brief   读取舵机当前位置
 * @details 查询舵机的实时位置信息，用于状态反馈和位置验证
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   position - 指向存储位置的指针，范围0-1000
 * @return  0表示成功读取，-1表示通信失败
 * @note    返回的position值范围为0-1000，对应舵机的全行程
 * ============================================================================
 */
int serial_servo_read_position(SerialServoControllerTypeDef *self, uint32_t servo_id, int16_t *position)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_POS_READ);  // 初始化读位置命令
    cmd_frame_complete(&frame, 0);                                    // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {      // 发送并等待响应
        // 将两个byte合成16位position
        *position = (int16_t)BYTE_TO_HW(self->rx_frame.elements.args[1], self->rx_frame.elements.args[0]);
        return 0;                                                      // 返回成功
    }
    return -1;                                                         // 返回失败
}

/**
 * ============================================================================
 * @brief   停止舵机运动
 * @details 立即停止指定舵机的运动，舵机保持当前位置
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @return  无
 * @note    停止立即生效，舵机将锁定在接收到此命令时的位置
 * ============================================================================
 */
void serial_servo_stop(SerialServoControllerTypeDef *self, uint32_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_MOVE_STOP);  // 初始化停止命令
    cmd_frame_complete(&frame, 0);                                     // 完成帧，无参数
    self->serial_write_and_read(self, &frame, true);                   // 发送命令（只写）
}

/**
 * ============================================================================
 * @brief   设置舵机角度偏差
 * @details 通过微调偏差值来纠正舵机的机械零点偏差，提高控制精度
 * @param   self          - 指向舵机控制器的指针
 * @param   servo_id      - 目标舵机的ID
 * @param   new_deviation - 新的偏差值，范围-125到+125
 * @return  无
 * @note    偏差值为有符号整数，需要调用serial_servo_save_deviation()才能永久保存
 * ============================================================================
 */
void serial_servo_set_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id, int new_deviation)
{
    SerialServoCmdTypeDef frame;
    // 限制deviation在有效范围[-125, +125]
    if (new_deviation > 125)  new_deviation = 125;      // 上限限制
    if (new_deviation < -125) new_deviation = -125;     // 下限限制
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_OFFSET_ADJUST);  // 初始化偏差调整命令
    frame.elements.args[0] = (uint8_t)((int8_t)new_deviation);  // 转为8位有符号整数
    cmd_frame_complete(&frame, 1);                              // 完成帧，1个参数
    self->serial_write_and_read(self, &frame, true);            // 发送命令（只写）
}

/**
 * ============================================================================
 * @brief   读取舵机角度偏差
 * @details 读取舵机当前的角度偏差值，用于验证零点校准状态
 * @param   self      - 指向舵机控制器的指针
 * @param   servo_id  - 目标舵机的ID
 * @param   deviation - 指向存储偏差值的指针
 * @return  0表示成功读取，-1表示通信失败
 * @note    返回值范围与设置范围一致：-125到+125
 * ============================================================================
 */
int serial_servo_read_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id, int8_t *deviation)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_OFFSET_READ);  // 初始化读偏差命令
    cmd_frame_complete(&frame, 0);                                            // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {              // 发送并等待响应
        *deviation = (int8_t)self->rx_frame.elements.args[0];                 // 转为8位有符号整数
        return 0;                                                              // 返回成功
    }
    return -1;                                                                 // 返回失败
}

/**
 * ============================================================================
 * @brief   保存舵机角度偏差到EEPROM
 * @details 将当前的角度偏差值永久保存到舵机的非易失性存储器中
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @return  无
 * @note    必须先调用serial_servo_set_deviation()设置偏差，再调用本函数保存
 * ============================================================================
 */
void serial_servo_save_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_OFFSET_WRITE);  // 初始化保存偏差命令
    cmd_frame_complete(&frame, 0);                                             // 完成帧，无参数
    self->serial_write_and_read(self, &frame, true);                           // 发送命令（只写）
}

/**
 * ============================================================================
 * @brief   设置舵机负载状态（锁定/解锁）
 * @details 控制舵机电机的供电状态，锁定时舵机保持当前位置，解锁时舵机可被外力转动
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   load     - 负载状态：0表示解锁(无扭矩)，1表示锁定(有扭矩)
 * @return  无
 * @note    解锁时舵机不会保持位置，但会降低功耗和发热
 * ============================================================================
 */
void serial_servo_load_unload(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t load)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE);  // 初始化负载设置命令
    frame.elements.args[0] = (uint8_t)load;  // 0=解锁, 1=锁定
    cmd_frame_complete(&frame, 1);           // 完成帧，1个参数
    self->serial_write_and_read(self, &frame, true);  // 发送命令（只写）
}

/**
 * ============================================================================
 * @brief   读取舵机负载状态
 * @details 查询舵机当前的锁定/解锁状态
 * @param   self       - 指向舵机控制器的指针
 * @param   servo_id   - 目标舵机的ID
 * @param   load_unload - 指向存储状态的指针：0表示解锁，1表示锁定
 * @return  0表示成功读取，-1表示通信失败
 * ============================================================================
 */
int serial_servo_read_load_unload(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t* load_unload)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_READ);  // 初始化读负载命令
    cmd_frame_complete(&frame, 0);                                              // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {                // 发送并等待响应
        *load_unload = self->rx_frame.elements.args[0];                         // 提取负载状态
        return 0;                                                                // 返回成功
    }
    return -1;                                                                   // 返回失败
}

/**
 * ============================================================================
 * @brief   设置舵机角度限制范围
 * @details 限制舵机的活动范围，舵机无法超出设定的角度界限，提供机械保护
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   limit_l  - 下限位置，范围0-1000
 * @param   limit_h  - 上限位置，范围0-1000
 * @return  无
 * @note    自动纠正参数大小关系（若limit_l > limit_h则自动交换）；
 *          值限制在0-1000范围内；需保存才能永久有效
 * ============================================================================
 */
void serial_servo_set_angle_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    // 限制所有值在0-1000范围内
    if (limit_l > 1000) limit_l = 1000;
    if (limit_h > 1000) limit_h = 1000;
    
    // 自动纠正上下限关系，确保limit_l < limit_h
    uint32_t real_limit_l = (limit_l > limit_h) ? limit_h : limit_l;  // 确保为较小值
    uint32_t real_limit_h = (limit_l > limit_h) ? limit_l : limit_h;  // 确保为较大值
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_LIMIT_WRITE);  // 初始化角度限制命令
    frame.elements.args[0] = GET_LOW_BYTE(real_limit_l);    // 下限低byte
    frame.elements.args[1] = GET_HIGH_BYTE(real_limit_l);   // 下限高byte
    frame.elements.args[2] = GET_LOW_BYTE(real_limit_h);    // 上限低byte
    frame.elements.args[3] = GET_HIGH_BYTE(real_limit_h);   // 上限高byte
    cmd_frame_complete(&frame, 4);                          // 完成帧，4个参数
    self->serial_write_and_read(self, &frame, true);        // 发送命令（只写）
}

/**
 * ============================================================================
 * @brief   读取舵机角度限制范围
 * @details 查询舵机当前设定的角度限制值
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   limit    - 指向数组的指针，[0]存储下限，[1]存储上限
 * @return  0表示成功读取，-1表示通信失败
 * @note    返回数组中 limit[0]=下限, limit[1]=上限
 * ============================================================================
 */
int serial_servo_read_angle_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_LIMIT_READ);  // 初始化读角度限制命令
    cmd_frame_complete(&frame, 0);                                            // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {              // 发送并等待响应
        // 组合两个byte为16位值，存储下限
        limit[0] = BYTE_TO_HW(self->rx_frame.elements.args[1], self->rx_frame.elements.args[0]);
        // 组合两个byte为16位值，存储上限
        limit[1] = BYTE_TO_HW(self->rx_frame.elements.args[3], self->rx_frame.elements.args[2]);
        return 0;                                                              // 返回成功
    }
    return -1;                                                                 // 返回失败
}

/**
 * ============================================================================
 * @brief   设置舵机温度上限
 * @details 设置舵机的最高允许工作温度，超过此温度舵机会自动降速或停止工作
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   limit    - 温度上限值，单位度C，范围50-100
 * @return  无
 * @note    超过上限温度舵机会降速运行以降温；值会自动限制在50-100范围
 * ============================================================================
 */
void serial_servo_set_temp_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit)
{
    SerialServoCmdTypeDef frame;
    // 限制温度在50-100°C范围内
    if (limit > 100) limit = 100;  // 上限限制
    if (limit < 50)  limit = 50;   // 下限限制
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE);  // 初始化温度限制命令
    frame.elements.args[0] = (uint8_t)limit;  // 温度值
    cmd_frame_complete(&frame, 1);            // 完成帧，1个参数
    self->serial_write_and_read(self, &frame, true);  // 发送命令（只写）
}

/**
 * ============================================================================
 * @brief   读取舵机温度上限
 * @details 查询舵机当前设定的最高工作温度
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   limit    - 指向存储限制值的指针，单位度C
 * @return  0表示成功读取，-1表示通信失败
 * ============================================================================
 */
int serial_servo_read_temp_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *limit)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_READ);  // 初始化读温度限制命令
    cmd_frame_complete(&frame, 0);                                             // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {               // 发送并等待响应
        *limit = self->rx_frame.elements.args[0];                             // 提取温度限制值
        return 0;                                                              // 返回成功
    }
    return -1;                                                                 // 返回失败
}

/**
 * ============================================================================
 * @brief   读取舵机当前温度
 * @details 查询舵机内部的实时温度值，用于温度监测和过温保护
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   temp     - 指向存储温度值的指针，单位度C
 * @return  0表示成功读取，-1表示通信失败
 * @note    返回值为uint8_t，范围0-255°C，典型工作温度5-60°C
 * ============================================================================
 */
int serial_servo_read_temp(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *temp)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_TEMP_READ);  // 初始化读温度命令
    cmd_frame_complete(&frame, 0);                                     // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {       // 发送并等待响应
        *temp = self->rx_frame.elements.args[0];                       // 提取温度值
        return 0;                                                       // 返回成功
    }
    return -1;                                                          // 返回失败
}

/**
 * ============================================================================
 * @brief   设置舵机输入电压限制范围
 * @details 限制舵机允许的工作电压范围，超出范围舵机会自动降速或停止，保护硬件
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   limit_l  - 下限电压，单位:0.1V，范围4500-14000(即4.5V-14V)
 * @param   limit_h  - 上限电压，单位:0.1V，范围4500-14000(即4.5V-14V)
 * @return  无
 * @note    自动纠正参数大小关系；值会自动限制在4500-14000范围；
 *          电压值单位为0.1V，如limit_l=5000代表5V
 * ============================================================================
 */
void serial_servo_set_vin_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    // 限制所有值在4500-14000范围内(4.5V-14V)
    if (limit_l < 4500)  limit_l = 4500;    // 下限保护
    if (limit_l > 14000) limit_l = 14000;   // 上限保护
    if (limit_h < 4500)  limit_h = 4500;    // 下限保护
    if (limit_h > 14000) limit_h = 14000;   // 上限保护
    
    // 自动纠正上下限关系，确保real_limit_l < real_limit_h
    uint32_t real_limit_l = (limit_l > limit_h) ? limit_h : limit_l;  // 确保为较小值
    uint32_t real_limit_h = (limit_l > limit_h) ? limit_l : limit_h;  // 确保为较大值
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_VIN_LIMIT_WRITE);  // 初始化电压限制命令
    frame.elements.args[0] = GET_LOW_BYTE(real_limit_l);    // 下限低byte
    frame.elements.args[1] = GET_HIGH_BYTE(real_limit_l);   // 下限高byte
    frame.elements.args[2] = GET_LOW_BYTE(real_limit_h);    // 上限低byte
    frame.elements.args[3] = GET_HIGH_BYTE(real_limit_h);   // 上限高byte
    cmd_frame_complete(&frame, 4);                          // 完成帧，4个参数
    self->serial_write_and_read(self, &frame, true);        // 发送命令（只写）
}

/**
 * ============================================================================
 * @brief   读取舵机电压限制范围
 * @details 查询舵机当前设定的输入电压限制值
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   limit    - 指向数组的指针，[0]存储下限，[1]存储上限，单位:0.1V
 * @return  0表示成功读取，-1表示通信失败
 * @note    返回值单位为0.1V，如值5000表示5V；limit[0]=下限，limit[1]=上限
 * ============================================================================
 */
int serial_servo_read_vin_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_VIN_LIMIT_READ);  // 初始化读电压限制命令
    cmd_frame_complete(&frame, 0);                                         // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {           // 发送并等待响应
        // 组合两个byte为16位值，存储下限
        limit[0] = BYTE_TO_HW(self->rx_frame.elements.args[1], self->rx_frame.elements.args[0]);
        // 组合两个byte为16位值，存储上限
        limit[1] = BYTE_TO_HW(self->rx_frame.elements.args[3], self->rx_frame.elements.args[2]);
        return 0;                                                           // 返回成功
    }
    return -1;                                                              // 返回失败
}

/**
 * ============================================================================
 * @brief   读取舵机当前输入电压
 * @details 查询舵机的实时输入电压值，用于电源监测和故障诊断
 * @param   self     - 指向舵机控制器的指针
 * @param   servo_id - 目标舵机的ID
 * @param   vin      - 指向存储电压值的指针，单位:0.1V
 * @return  0表示成功读取，-1表示通信失败
 * @note    返回值单位为0.1V，如vin=5000表示5.0V；典型范围5000-12000(5V-12V)
 * ============================================================================
 */
int serial_servo_read_vin(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t *vin)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_VIN_READ);  // 初始化读电压命令
    cmd_frame_complete(&frame, 0);                                    // 完成帧，无参数
    if (0 == self->serial_write_and_read(self, &frame, false)) {      // 发送并等待响应
        // 组合两个byte为16位电压值
        *vin = BYTE_TO_HW(self->rx_frame.elements.args[1], self->rx_frame.elements.args[0]);
        return 0;                                                      // 返回成功
    }
    return -1;                                                         // 返回失败
}

/**
 * ============================================================================
 * @brief   舵机控制器对象初始化
 * @details 初始化舵机控制器的所有成员变量，包括状态机、缓冲区和回调函数
 * @param   self - 指向舵机控制器结构体的指针
 * @return  无
 * @note    需要在使用前调用此函数；初始化后需要设置serial_write_and_read回调函数
 * ============================================================================
 */
void serial_servo_controller_object_init(SerialServoControllerTypeDef *self)
{
    self->proc_timeout = 8;                                    // 设置处理超时时间(单位待定)
    self->rx_args_index = 0;                                   // 清零接收参数索引
    self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;           // 初始化接收状态为等待第一个帧头
    self->rx_completed = false;                                // 接收完成标志置为false
    memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef)); // 清空接收帧缓冲区
    self->tx_only = true;                                      // 设置为只发送模式
    self->tx_byte_index = 0;                                   // 清零发送byte索引
    memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef)); // 清空发送帧缓冲区
    self->serial_write_and_read = NULL;                        // 清空回调函数指针(需要后续赋值)
}
