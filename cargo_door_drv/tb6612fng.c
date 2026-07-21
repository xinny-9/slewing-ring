#include "tb6612fng.h"
#include <stdio.h>

/**
  * @brief  控制电机状态与速度
  * @param  motor: 电机结构体指针
  * @param  state: 电机状态 (MOTOR_STOP, MOTOR_FWD, MOTOR_REV, MOTOR_BRAKE)
  * @param  speed: PWM 占空比值 (通常从 0 到 ARR 的最大值)
  */
void TB6612_SetMotor(TB6612_MotorTypeDef* motor, uint8_t state, uint16_t speed) {
    // 调试反馈打印
    printf("  [HW_SET] Motor Pins config: IN1_Pin=%d, IN2_Pin=%d | SetState=%d, SetSpeed=%d\r\n", 
           motor->IN1_Pin, motor->IN2_Pin, state, speed);
    
    switch (state) {
        case MOTOR_FWD:  // 正转: IN1=H, IN2=L
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, speed);
            break;
            
        case MOTOR_REV:  // 反转: IN1=L, IN2=H
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, speed);
            break;
            
        case MOTOR_STOP: // 停止 (滑行停止): IN1=L, IN2=L
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);
            break;
            
        case MOTOR_BRAKE: // 短刹车: IN1=H, IN2=H
            HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0); 
            break;
    }
}
