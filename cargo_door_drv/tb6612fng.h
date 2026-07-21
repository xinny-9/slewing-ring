#ifndef __TB6612FNG_H
#define __TB6612FNG_H

#include "stm32f1xx_hal.h"

/* 电机状态宏定义 */
#define MOTOR_STOP    0  // 停止 (IN1=L, IN2=L)
#define MOTOR_FWD     1  // 正转 / CW (IN1=H, IN2=L)
#define MOTOR_REV     2  // 反转 / CCW (IN1=L, IN2=H)
#define MOTOR_BRAKE   3  // 短刹车 (IN1=H, IN2=H)

/* 电机结构体定义 */
typedef struct {
    GPIO_TypeDef* IN1_Port;    // IN1 GPIO 端口
    uint16_t           IN1_Pin;     // IN1 GPIO 引脚
    GPIO_TypeDef* IN2_Port;    // IN2 GPIO 端口
    uint16_t           IN2_Pin;     // IN2 GPIO 引脚
    TIM_HandleTypeDef* pwm_timer;   // PWM 对应的定时器句柄 (如 &htim1)
    uint32_t           pwm_channel; // PWM 对应的定时器通道 (如 TIM_CHANNEL_1)
} TB6612_MotorTypeDef;

/* 函数声明 */
void TB6612_SetMotor(TB6612_MotorTypeDef* motor, uint8_t state, uint16_t speed);

#endif /* __TB6612FNG_H */
