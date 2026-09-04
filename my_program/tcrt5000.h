#include <stdio.h>
#include "gpio.h"
#include "pinctrl.h"

void tcrt_config(uint16_t TCRT5000_DO_PIN, uint16_t TCRT5000_MODE);
uint8_t tcrt_check(uint16_t TCRT5000_DO_PIN);