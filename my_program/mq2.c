/*
 * MQ-2 combustible gas sensor driver for WS63
 *
 * MQ-2 detects: LPG, propane, hydrogen, methane, alcohol, smoke
 * Output: analog voltage proportional to gas concentration
 * Clean air: output is low (< 300mV typical)
 * Gas detected: output rises with concentration
 */

#include "mq2.h"
#include "adc.h"
#include "adc_porting.h"
#include "soc_osal.h"

static uint8_t g_mq2_channel = 0;

int mq2_init(uint8_t channel)
{
    g_mq2_channel = channel;

    /* Initialize ADC */
    if (uapi_adc_init(ADC_CLOCK_NONE) != ERRCODE_SUCC) {
        osal_printk("MQ2: ADC init failed\r\n");
        return -1;
    }

    osal_printk("MQ2: init ok, channel=%d\r\n", channel);
    return 0;
}

int mq2_read_voltage(uint16_t *voltage_mv)
{
    if (adc_port_read(g_mq2_channel, voltage_mv) != ERRCODE_SUCC) {
        return -1;
    }
    return 0;
}

mq2_level_t mq2_get_level(uint16_t voltage_mv)
{
    /*
     * MQ-2 voltage thresholds (adjust based on your circuit):
     * - Clean air: 0 - 300mV
     * - Light gas: 300 - 600mV
     * - Medium gas: 600 - 1000mV
     * - High gas: > 1000mV
     *
     * These values assume the sensor output is within ADC range (0-1.8V).
     * If your board has a voltage divider or different Vref, adjust accordingly.
     */
    if (voltage_mv < 300) {
        return MQ2_LEVEL_CLEAN;
    } else if (voltage_mv < 600) {
        return MQ2_LEVEL_LIGHT;
    } else if (voltage_mv < 1000) {
        return MQ2_LEVEL_MEDIUM;
    } else {
        return MQ2_LEVEL_HIGH;
    }
}

const char *mq2_level_string(mq2_level_t level)
{
    switch (level) {
    case MQ2_LEVEL_CLEAN:
        return "Clean";
    case MQ2_LEVEL_LIGHT:
        return "Light";
    case MQ2_LEVEL_MEDIUM:
        return "Medium";
    case MQ2_LEVEL_HIGH:
        return "HIGH!";
    default:
        return "Unknown";
    }
}