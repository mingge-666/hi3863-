#include "pinctrl.h"
#include "common_def.h"
#include "gpio.h"
#include "pwm.h"
errcode_t pwm_control(uint16_t PWM_PIN, uint16_t PWM_MODE,uint16_t PWM_LOW_TIME, 
    uint16_t PWM_HIGH_TIME, uint16_t PWM_CHANNEL);
void motor_move_config(uint16_t MOTOR_MOVE_PIN, uint16_t MOTOR_MOVE_PIN_MODE);//电机驱动引脚初始化和配置
void motor_stop(uint16_t LEFT_MOTOR_MOVE_OUT1, uint16_t LEFT_MOTOR_MOVE_OUT2,
    uint16_t RIGHT_MOTOR_MOVE_OUT1, uint16_t RIGHT_MOTOR_MOVE_OUT2);//电机停止函数
void motor_left_forward(uint16_t LEFT_MOTOR_MOVE_OUT1, uint16_t LEFT_MOTOR_MOVE_OUT2);
void motor_left_backward(uint16_t LEFT_MOTOR_MOVE_OUT1, uint16_t LEFT_MOTOR_MOVE_OUT2);
void motor_right_forward(uint16_t RIGHT_MOTOR_MOVE_OUT1, uint16_t RIGHT_MOTOR_MOVE_OUT2);
void motor_right_backward(uint16_t RIGHT_MOTOR_MOVE_OUT1, uint16_t RIGHT_MOTOR_MOVE_OUT2);