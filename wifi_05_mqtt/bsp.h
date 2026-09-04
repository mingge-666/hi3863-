#ifndef BSP_H
#define BSP_H

#include "pinctrl.h"
#include "gpio.h"
#include "platform_core_rom.h"

#define SENSOR_IO GPIO_10

void my_gpio_init(pin_t gpio);

gpio_level_t my_io_readval(pin_t gpio);

void my_io_setval(pin_t gpio, gpio_level_t level);
#endif