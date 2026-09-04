#include "pinctrl.h"
#include "common_def.h"
#include "gpio.h"
void led_config(uint16_t BLINKY_PIN, uint16_t BLINKY_PIN_MODE);
void blinky_open(uint16_t BLINKY_PIN, uint16_t GPIO_LEVEL_HIGH);
void blinky_close(uint16_t BLINKY_PIN, uint16_t GPIO_LEVEL_LOW);