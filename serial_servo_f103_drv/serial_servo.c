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

static void cmd_frame_init(SerialServoCmdTypeDef *frame, uint8_t servo_id, uint8_t cmd)
{
    frame->header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame->header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame->elements.servo_id = servo_id;
    frame->elements.command = cmd;
}

static void cmd_frame_complete(SerialServoCmdTypeDef *frame, uint8_t args_num)
{
    frame->elements.length = args_num + 3;
    frame->elements.args[args_num] = serial_servo_checksum((uint8_t*)frame);
}

void serial_servo_set_id(SerialServoControllerTypeDef *self, uint32_t old_id, uint32_t new_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)old_id, SERIAL_SERVO_ID_WRITE);
    frame.elements.args[0] = (uint8_t)new_id;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true); 
}

int serial_servo_read_id(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *ret_servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ID_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        *ret_servo_id = self->rx_frame.elements.args[0];
        return 0;
    }
    return -1;
}

void serial_servo_set_position(SerialServoControllerTypeDef *self, uint32_t servo_id, int position, uint32_t duration)
{
    SerialServoCmdTypeDef frame;
    if (position > 1000) position = 1000;
    if (position < 0)    position = 0;
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_MOVE_TIME_WRITE);
    frame.elements.args[0] = GET_LOW_BYTE(position);
    frame.elements.args[1] = GET_HIGH_BYTE(position);
    frame.elements.args[2] = GET_LOW_BYTE(duration);
    frame.elements.args[3] = GET_HIGH_BYTE(duration);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true); 
}

int serial_servo_read_position(SerialServoControllerTypeDef *self, uint32_t servo_id, int16_t *position)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_POS_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        *position = (int16_t)BYTE_TO_HW(self->rx_frame.elements.args[1], self->rx_frame.elements.args[0]);
        return 0;
    }
    return -1;
}

void serial_servo_stop(SerialServoControllerTypeDef *self, uint32_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_MOVE_STOP);
    cmd_frame_complete(&frame, 0);
    self->serial_write_and_read(self, &frame, true);
}

void serial_servo_set_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id, int new_deviation)
{
    SerialServoCmdTypeDef frame;
    if (new_deviation > 125)  new_deviation = 125;
    if (new_deviation < -125) new_deviation = -125;
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_OFFSET_ADJUST);
    frame.elements.args[0] = (uint8_t)((int8_t)new_deviation);
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

int serial_servo_read_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id, int8_t *deviation)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_OFFSET_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        *deviation = (int8_t)self->rx_frame.elements.args[0];
        return 0;
    }
    return -1;
}

void serial_servo_save_deviation(SerialServoControllerTypeDef *self, uint32_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_OFFSET_WRITE);
    cmd_frame_complete(&frame, 0);
    self->serial_write_and_read(self, &frame, true);
}

void serial_servo_load_unload(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t load)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE);
    frame.elements.args[0] = (uint8_t)load;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

int serial_servo_read_load_unload(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t* load_unload)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        *load_unload = self->rx_frame.elements.args[0];
        return 0;
    }
    return -1;
}

void serial_servo_set_angle_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    if (limit_l > 1000) limit_l = 1000;
    if (limit_h > 1000) limit_h = 1000;
    
    uint32_t real_limit_l = (limit_l > limit_h) ? limit_h : limit_l;
    uint32_t real_limit_h = (limit_l > limit_h) ? limit_l : limit_h;
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_LIMIT_WRITE);
    frame.elements.args[0] = GET_LOW_BYTE(real_limit_l);
    frame.elements.args[1] = GET_HIGH_BYTE(real_limit_l);
    frame.elements.args[2] = GET_LOW_BYTE(real_limit_h);
    frame.elements.args[3] = GET_HIGH_BYTE(real_limit_h);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true);
}

int serial_servo_read_angle_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_ANGLE_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        limit[0] = BYTE_TO_HW(self->rx_frame.elements.args[1], self->rx_frame.elements.args[0]);
        limit[1] = BYTE_TO_HW(self->rx_frame.elements.args[3], self->rx_frame.elements.args[2]);
        return 0;
    }
    return -1;
}

void serial_servo_set_temp_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit)
{
    SerialServoCmdTypeDef frame;
    if (limit > 100) limit = 100;
    if (limit < 50)  limit = 50;
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE);
    frame.elements.args[0] = (uint8_t)limit;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

int serial_servo_read_temp_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *limit)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        *limit = self->rx_frame.elements.args[0];
        return 0;
    }
    return -1;
}

int serial_servo_read_temp(SerialServoControllerTypeDef *self, uint32_t servo_id, uint8_t *temp)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_TEMP_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        *temp = self->rx_frame.elements.args[0];
        return 0;
    }
    return -1;
}

void serial_servo_set_vin_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    if (limit_l < 4500)  limit_l = 4500;
    if (limit_l > 14000) limit_l = 14000;
    if (limit_h < 4500)  limit_h = 4500;
    if (limit_h > 14000) limit_h = 14000;
    
    uint32_t real_limit_l = (limit_l > limit_h) ? limit_h : limit_l;
    uint32_t real_limit_h = (limit_l > limit_h) ? limit_l : limit_h;
    
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_VIN_LIMIT_WRITE);
    frame.elements.args[0] = GET_LOW_BYTE(real_limit_l);
    frame.elements.args[1] = GET_HIGH_BYTE(real_limit_l);
    frame.elements.args[2] = GET_LOW_BYTE(real_limit_h);
    frame.elements.args[3] = GET_HIGH_BYTE(real_limit_h);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true);
}

int serial_servo_read_vin_limit(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_VIN_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        limit[0] = BYTE_TO_HW(self->rx_frame.elements.args[1], self->rx_frame.elements.args[0]);
        limit[1] = BYTE_TO_HW(self->rx_frame.elements.args[3], self->rx_frame.elements.args[2]);
        return 0;
    }
    return -1;
}

int serial_servo_read_vin(SerialServoControllerTypeDef *self, uint32_t servo_id, uint16_t *vin)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, (uint8_t)servo_id, SERIAL_SERVO_VIN_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) {
        *vin = BYTE_TO_HW(self->rx_frame.elements.args[1], self->rx_frame.elements.args[0]);
        return 0;
    }
    return -1;
}

void serial_servo_controller_object_init(SerialServoControllerTypeDef *self)
{
    self->proc_timeout = 8;  
    self->rx_args_index = 0;
    self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
    self->rx_completed = false;
    memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
    self->tx_only = true;
    self->tx_byte_index = 0;
    memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
    self->serial_write_and_read = NULL; 
}
