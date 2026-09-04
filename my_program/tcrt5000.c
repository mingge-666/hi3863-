#include <stdio.h>
#include "cmsis_os2.h"
#include "gpio.h"
#include "soc_osal.h"
#include "pinctrl.h"
#include "common_def.h"


/* 初始化 */
void tcrt_config(uint16_t TCRT5000_DO_PIN, uint16_t TCRT5000_MODE)
{
    uapi_pin_init();
    uapi_pin_set_mode(TCRT5000_DO_PIN, TCRT5000_MODE);
    uapi_gpio_set_dir(TCRT5000_DO_PIN, GPIO_DIRECTION_INPUT);
    uapi_gpio_set_val(TCRT5000_DO_PIN, GPIO_LEVEL_LOW);
}

/* 读取是否为黑色 */
uint8_t tcrt_check(uint16_t TCRT5000_DO_PIN)
{
    uint8_t val = 0;
    val=uapi_gpio_get_val(TCRT5000_DO_PIN);
    return val;  // 1: 黑线, 0: 白色
}

