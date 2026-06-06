/**
 * *****************************************************************************
 * @file    serial_servo.h
 * @author  Antigravity
 * @brief   串口总线舵机核心协议与控制器定义头文件 (STM32F103C8T6 裸机优化版)
 * *****************************************************************************
 */

#ifndef __SERIAL_SERVO_H
#define __SERIAL_SERVO_H

#pragma anon_unions

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* --- 幻尔总线舵机通信协议指令定义 --- */
#define SERIAL_SERVO_FRAME_HEADER         0x55    // 帧头 (连续两个 0x55)
#define SERIAL_SERVO_MOVE_TIME_WRITE      1       // 写入舵机位置与运行时间
#define SERIAL_SERVO_MOVE_TIME_READ       2       // 读取舵机位置与运行时间
#define SERIAL_SERVO_MOVE_TIME_WAIT_WRITE 7       // 写入舵机预备运行位置与时间
#define SERIAL_SERVO_MOVE_TIME_WAIT_READ  8       // 读取舵机预备运行位置与时间
#define SERIAL_SERVO_MOVE_START           11      // 启动预备运行的舵机
#define SERIAL_SERVO_MOVE_STOP            12      // 紧急停止舵机
#define SERIAL_SERVO_ID_WRITE             13      // 写入舵机ID
#define SERIAL_SERVO_ID_READ              14      // 读取舵机ID
#define SERIAL_SERVO_ANGLE_OFFSET_ADJUST  17      // 调整舵机偏差(临时)
#define SERIAL_SERVO_ANGLE_OFFSET_WRITE   18      // 保存舵机偏差(固化)
#define SERIAL_SERVO_ANGLE_OFFSET_READ    19      // 读取舵机偏差
#define SERIAL_SERVO_ANGLE_LIMIT_WRITE    20      // 写入角度限制范围
#define SERIAL_SERVO_ANGLE_LIMIT_READ     21      // 读取角度限制范围
#define SERIAL_SERVO_VIN_LIMIT_WRITE      22      // 写入电压限制范围
#define SERIAL_SERVO_VIN_LIMIT_READ       23      // 读取电压限制范围
#define SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE 24      // 写入最高温度限制
#define SERIAL_SERVO_TEMP_MAX_LIMIT_READ  25      // 读取最高温度限制
#define SERIAL_SERVO_TEMP_READ            26      // 读取当前温度
#define SERIAL_SERVO_VIN_READ             27      // 读取当前电压
#define SERIAL_SERVO_POS_READ             28      // 读取当前角度/位置
#define SERIAL_SERVO_OR_MOTOR_MODE_WRITE  29      // 写入舵机或电机工作模式
#define SERIAL_SERVO_OR_MOTOR_MODE_READ   30      // 读取工作模式
#define SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE 31      // 舵机掉电释放/上电锁砖
#define SERIAL_SERVO_LOAD_OR_UNLOAD_READ  32      // 读取舵机扭矩开关状态
#define SERIAL_SERVO_LED_CTRL_WRITE       33      // 写入LED控制开关
#define SERIAL_SERVO_LED_CTRL_READ        34      // 读取LED控制开关
#define SERIAL_SERVO_LED_ERROR_WRITE      35      // 写入LED报警故障类型
#define SERIAL_SERVO_LED_ERROR_READ       36      // 读取LED报警故障类型

#define CMD_SERVO_MOVE 0x03

#pragma pack(1)
/**
 * @brief 串口总线舵机通信数据帧结构体
 */
typedef struct {
    uint8_t header_1;   // 帧头1 (0x55)
    uint8_t header_2;   // 帧头2 (0x55)
    union {
        struct {
            uint8_t servo_id;   // 舵机ID (0~253, 254为广播ID)
            uint8_t length;     // 数据长度 (等于：参数个数 + 3)
            uint8_t command;    // 控制指令
            uint8_t args[8];    // 缓存参数与最后的校验和字节 (最大长度为8)
        } elements;
        uint8_t data_raw[11];   // 原始一维数组，方便物理层按字节收发与校验
    };
} SerialServoCmdTypeDef;
#pragma pack()

/**
 * @brief 串口接收状态机枚举
 */
typedef enum {
    SERIAL_SERVO_RECV_STARTBYTE_1,  // 寻找帧头1 (0x55)
    SERIAL_SERVO_RECV_STARTBYTE_2,  // 寻找帧头2 (0x55)
    SERIAL_SERVO_RECV_SERVO_ID,     // 接收舵机ID
    SERIAL_SERVO_RECV_LENGTH,       // 接收数据长度
    SERIAL_SERVO_RECV_COMMAND,      // 接收指令码
    SERIAL_SERVO_RECV_ARGUMENTS,    // 接收参数数据
    SERIAL_SERVO_RECV_CHECKSUM,     // 接收校验和
} SerialServoRecvState;

/* 前向声明控制器结构体 */
typedef struct SerialServoControllerTypeDef SerialServoControllerTypeDef;

/**
 * @brief 总线舵机控制器核心管理结构体
 */
struct SerialServoControllerTypeDef {
    SerialServoRecvState rx_state;     // 串口接收解析状态机状态
    SerialServoCmdTypeDef rx_frame;    // 接收到的数据帧
    uint32_t rx_args_index;            // 参数接收计数器
    volatile bool rx_completed;        // 接收完成标志 (整帧校验通过后置位)

    SerialServoCmdTypeDef tx_frame;    // 准备发送的数据帧
    uint32_t tx_byte_index;            // 字节发送计数器
    bool tx_only;                      // 本次指令是否为单向发送(无需等待舵机回传数据)

    uint32_t proc_timeout;             // 物理层等待应答的超时时间限制 (单位：ms)

    /**
     * @brief 物理层驱动写与读的回调函数指针
     * @param self    控制器结构体指针
     * @param frame   待发送的数据帧指针
     * @param tx_only 本次操作是否仅发送不接收
     * @retval 0: 成功, -1: 物理层等待接收超时, 其他: 硬件故障
     */
    int (*serial_write_and_read)(SerialServoControllerTypeDef *self, SerialServoCmdTypeDef *frame, bool tx_only);
};

/* --- 核心控制与配置API函数声明 --- */
void serial_servo_controller_object_init(SerialServoControllerTypeDef *self);
void serial_servo_set_id(SerialServoControllerTypeDef *self, uint32_t old_id, uint32_t new_id);
int serial_servo_read_id(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *ret_servo_id);
void serial_servo_set_position(SerialServoControllerTypeDef *self, uint32_t servo_id, int position, uint32_t duration);
int serial_servo_read_position(SerialServoControllerTypeDef *self, uint32_t servo_id, int16_t *position);
void serial_servo_stop(SerialServoControllerTypeDef *self, uint32_t servo_id);
void serial_servo_set_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id, int new_deviation);
int serial_servo_read_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id, int8_t *deviation);
void serial_servo_save_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id);
void serial_servo_load_unload(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t load);
int serial_servo_read_load_unload(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t* load_unload);
void serial_servo_set_angle_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h);
int serial_servo_read_angle_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t limit[2]);
void serial_servo_set_temp_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit);
int serial_servo_read_temp_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *limit);
int serial_servo_read_temp(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *temp);
void serial_servo_set_vin_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h);
int serial_servo_read_vin_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t limit[2]);
int serial_servo_read_vin(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t *vin);

/**
 * @brief 计算总线舵机协议帧的校验和 (Checksum)
 * @param buf 数据包起始地址
 * @return 计算出的1字节校验和值
 */
static inline uint8_t serial_servo_checksum(const uint8_t buf[])
{
    uint16_t temp = 0;
    for (int i = 2; i < buf[3] + 2; ++i) {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

/**
 * @brief 串口总线舵机中断接收解析状态机处理器
 */
static inline int serial_servo_rx_handler(SerialServoControllerTypeDef *self, uint8_t rx_byte)
{
    switch (self->rx_state) {
        case SERIAL_SERVO_RECV_STARTBYTE_1: {
            if (rx_byte == SERIAL_SERVO_FRAME_HEADER) {
                self->rx_frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
                self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_2;
                return 1;
            }
            return -1;
        }
        case SERIAL_SERVO_RECV_STARTBYTE_2: {
            if (rx_byte == SERIAL_SERVO_FRAME_HEADER) {
                self->rx_frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
                self->rx_state = SERIAL_SERVO_RECV_SERVO_ID;
                return 2;
            }
            self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
            return -2;
        }
        case SERIAL_SERVO_RECV_SERVO_ID: {
            self->rx_frame.elements.servo_id = rx_byte;
            self->rx_state = SERIAL_SERVO_RECV_LENGTH;
            return 3;
        }
        case SERIAL_SERVO_RECV_LENGTH: {
            if (rx_byte < 3 || rx_byte > 7) { 
                self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
                return -3;
            }
            self->rx_frame.elements.length = rx_byte;
            self->rx_state = SERIAL_SERVO_RECV_COMMAND;
            return 4;
        }
        case SERIAL_SERVO_RECV_COMMAND: {
            self->rx_frame.elements.command = rx_byte;
            self->rx_args_index = 0;
            if (self->rx_frame.elements.length == 3) {
                self->rx_state = SERIAL_SERVO_RECV_CHECKSUM;
            } else {
                self->rx_state = SERIAL_SERVO_RECV_ARGUMENTS;
            }
            return 5;
        }
        case SERIAL_SERVO_RECV_ARGUMENTS: {
            self->rx_frame.elements.args[self->rx_args_index++] = rx_byte;
            if (self->rx_args_index + 3 == self->rx_frame.elements.length) {
                self->rx_state = SERIAL_SERVO_RECV_CHECKSUM;
            }
            return 6;
        }
        case SERIAL_SERVO_RECV_CHECKSUM: {
            self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1; 
            if (serial_servo_checksum((uint8_t*)&self->rx_frame) == rx_byte) {
                self->rx_completed = true; // 标志整帧成功接收并校验通过
                return 0; 
            } else {
                return -99; 
            }
        }
        default: {
            self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
            return -100;
        }
    }
}

#endif /* __SERIAL_SERVO_H */
