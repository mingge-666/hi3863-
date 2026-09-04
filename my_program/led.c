#include "pinctrl.h"
#include "soc_osal.h"
#include "app_init.h"
#include "gpio.h" 

void led_config(uint16_t BLINKY_PIN, uint16_t BLINKY_PIN_MODE)//led初始化和配置 
{
    // 初始化引脚控制，设置BLINKY_PIN为输出模式
    uapi_pin_set_mode(BLINKY_PIN, BLINKY_PIN_MODE);// 设置引脚模式
    uapi_gpio_set_dir(BLINKY_PIN, GPIO_DIRECTION_OUTPUT); 
}

void blinky_open(uint16_t BLINKY_PIN, uint16_t GPIO_LEVEL_HIGH)
{
    uapi_gpio_set_val(BLINKY_PIN, GPIO_LEVEL_HIGH);
    osal_printk("Blinky working.\r\n");

}
void blinky_close(uint16_t BLINKY_PIN, uint16_t GPIO_LEVEL_LOW)
{
    uapi_gpio_set_val(BLINKY_PIN, GPIO_LEVEL_LOW);
    osal_printk("Blinky stopped.\r\n");

}