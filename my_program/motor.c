/**
 * 简化版PWM单引脚输出示例
 */
#include "common_def.h"
#include "pinctrl.h"
#include "pwm.h"
#include "soc_osal.h"
#include "app_init.h"
#include "gpio.h"


void motor_move_config(uint16_t MOTOR_MOVE_PIN, uint16_t MOTOR_MOVE_PIN_MODE)//电机驱动引脚初始化和配置 
{
    // 初始化引脚控制，设置MOTOR_MOVE_PIN为输出模式
    uapi_pin_set_mode(MOTOR_MOVE_PIN, MOTOR_MOVE_PIN_MODE);// 设置引脚模式
    uapi_gpio_set_dir(MOTOR_MOVE_PIN, GPIO_DIRECTION_OUTPUT); 
}
/* PWM配置参数 */
/* PWM中断回调函数（可选，如果不需要中断可以去掉） */
static errcode_t pwm_callback(uint8_t channel)
{
    /* 这里可以添加中断处理代码 */
    unused(channel);
    return ERRCODE_SUCC;
}

//pwm控制函数，参数为引脚号、引脚模式、低电平时间、高电平时间和PWM通道号
errcode_t pwm_control(uint16_t PWM_PIN, uint16_t PWM_MODE,uint16_t PWM_LOW_TIME, 
    uint16_t PWM_HIGH_TIME, uint16_t PWM_CHANNEL)
{
        pwm_config_t pwm_cfg = {
        PWM_LOW_TIME,// 低电平时间
        PWM_HIGH_TIME,// 高电平时间
        0,//cycles: 周期数 = 0
        0xFF,// offset: 相位偏移
        true,           /* true: 重复模式，无限循环输出 */
    };

    uapi_pin_set_mode(PWM_PIN,PWM_MODE);
    uapi_pwm_open(PWM_CHANNEL, &pwm_cfg);
    uapi_pwm_register_interrupt(PWM_CHANNEL, pwm_callback);
    #ifdef CONFIG_PWM_USING_V151
    /* 如果是V151架构，需要配置PWM组 */
    uint8_t channel_id = PWM_CHANNEL;
    uapi_pwm_set_group(0, &channel_id, 1);  /* 使用组0，包含1个通道 */
    uapi_pwm_start_group(0);
#else
    /* 标准PWM启动方式 */
    uapi_pwm_start(PWM_CHANNEL);
#endif

    return ERRCODE_SUCC;
}

void motor_stop(uint16_t LEFT_MOTOR_MOVE_OUT1, uint16_t LEFT_MOTOR_MOVE_OUT2,uint16_t RIGHT_MOTOR_MOVE_OUT1, uint16_t RIGHT_MOTOR_MOVE_OUT2)//电机停止函数
{
    uapi_gpio_set_val(LEFT_MOTOR_MOVE_OUT1, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(LEFT_MOTOR_MOVE_OUT2, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(RIGHT_MOTOR_MOVE_OUT1, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(RIGHT_MOTOR_MOVE_OUT2, GPIO_LEVEL_LOW);
}
void motor_left_forward(uint16_t LEFT_MOTOR_MOVE_OUT1, uint16_t LEFT_MOTOR_MOVE_OUT2)//电机前进函数
{
    uapi_gpio_set_val(LEFT_MOTOR_MOVE_OUT1, GPIO_LEVEL_HIGH);
    uapi_gpio_set_val(LEFT_MOTOR_MOVE_OUT2, GPIO_LEVEL_LOW);
}
void motor_left_backward(uint16_t LEFT_MOTOR_MOVE_OUT1, uint16_t LEFT_MOTOR_MOVE_OUT2)//电机后退函数
{
    uapi_gpio_set_val(LEFT_MOTOR_MOVE_OUT1, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(LEFT_MOTOR_MOVE_OUT2, GPIO_LEVEL_HIGH);
}
void motor_right_forward(uint16_t RIGHT_MOTOR_MOVE_OUT1, uint16_t RIGHT_MOTOR_MOVE_OUT2)//电机前进函数
{
    uapi_gpio_set_val(RIGHT_MOTOR_MOVE_OUT1, GPIO_LEVEL_HIGH);
    uapi_gpio_set_val(RIGHT_MOTOR_MOVE_OUT2, GPIO_LEVEL_LOW);
}
void motor_right_backward(uint16_t RIGHT_MOTOR_MOVE_OUT1, uint16_t RIGHT_MOTOR_MOVE_OUT2)//电机后退函数
{
    uapi_gpio_set_val(RIGHT_MOTOR_MOVE_OUT1, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(RIGHT_MOTOR_MOVE_OUT2, GPIO_LEVEL_HIGH);
}

