/*
 * MQ-2 combustible gas sensor driver for WS63
 */

#ifndef MQ2_H
#define MQ2_H

#include <stdint.h>

/* MQ-2 sensor gas levels */
typedef enum {
    MQ2_LEVEL_CLEAN = 0,
    MQ2_LEVEL_LIGHT,
    MQ2_LEVEL_MEDIUM,
    MQ2_LEVEL_HIGH
} mq2_level_t;

/**
 * @brief Initialize MQ-2 sensor (ADC config)
 * @param channel ADC channel connected to MQ-2 AOUT
 * @return 0 on success, -1 on error
 */
int mq2_init(uint8_t channel);

/**
 * @brief Read MQ-2 sensor voltage
 * @param voltage_mv Output: voltage in millivolts
 * @return 0 on success, -1 on error
 */
int mq2_read_voltage(uint16_t *voltage_mv);

/**
 * @brief Get gas concentration level
 * @param voltage_mv Current sensor voltage in mV
 * @return Gas level enum
 */
mq2_level_t mq2_get_level(uint16_t voltage_mv);

/**
 * @brief Get gas level as readable string
 * @param level Gas level enum
 * @return String description
 */
const char *mq2_level_string(mq2_level_t level);

#endif /* MQ2_H */