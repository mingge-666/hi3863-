/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AHT20_H
#define AHT20_H

#include <stdint.h>

#define AHT20_I2C_ADDR 0x38

// AHT20 commands
#define AHT20_CMD_INIT 0xBE
#define AHT20_CMD_MEASURE 0xAC
#define AHT20_CMD_RESET 0xBA
#define AHT20_CMD_STATUS 0x71

// Status register bits
#define AHT20_STATUS_BUSY 0x80
#define AHT20_STATUS_CALIBRATED 0x08

/**
 * @brief Initialize AHT20 sensor
 * @return 0 on success, negative value on error
 */
int aht20_init(void);

/**
 * @brief Read temperature and humidity from AHT20
 * @param temperature Pointer to store temperature value (in Celsius * 100)
 * @param humidity Pointer to store humidity value (in % * 100)
 * @return 0 on success, negative value on error
 */
int aht20_read_temperature_humidity(int32_t *temperature, int32_t *humidity);

/**
 * @brief Check if AHT20 is calibrated
 * @return 1 if calibrated, 0 if not, negative value on error
 */
int aht20_is_calibrated(void);

/**
 * @brief Reset AHT20 sensor
 * @return 0 on success, negative value on error
 */
int aht20_reset(void);

#endif // AHT20_H