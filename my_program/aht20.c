/*
 * AHT20 driver with fixed data processing
 */

#include "aht20.h"
#include "i2c.h"
#include "soc_osal.h"

#define I2C_BUS_ID 1

static int aht20_write_cmd(const uint8_t *cmd, uint32_t len)
{
    i2c_data_t data = {0};
    data.send_buf = (uint8_t *)cmd;
    data.send_len = len;
    return uapi_i2c_master_write(I2C_BUS_ID, AHT20_I2C_ADDR, &data);
}

static int aht20_read_data(uint8_t *buf, uint32_t len)
{
    i2c_data_t data = {0};
    data.receive_buf = buf;
    data.receive_len = len;
    return uapi_i2c_master_read(I2C_BUS_ID, AHT20_I2C_ADDR, &data);
}

int aht20_init(void)
{
    uint8_t reset_cmd = AHT20_CMD_RESET;
    uint8_t init_cmd[2] = {AHT20_CMD_INIT, 0x00};
    uint8_t status;

    /* Step 1: Reset sensor - wait 20ms */
    if (aht20_write_cmd(&reset_cmd, 1) != ERRCODE_SUCC) {
        osal_printk("AHT20: reset failed\r\n");
        return -1;
    }
    osal_msleep(20);

    /* Step 2: Wait for sensor to power up - 40ms */
    osal_msleep(40);

    /* Step 3: Send init/calibrate command - wait 10ms */
    if (aht20_write_cmd(init_cmd, 2) != ERRCODE_SUCC) {
        osal_printk("AHT20: init cmd failed\r\n");
        return -1;
    }
    osal_msleep(10);

    /* Step 4: Wait for calibration - up to 100ms */
    osal_msleep(100);

    /* Step 5: Read status to verify calibration */
    status = AHT20_CMD_STATUS;
    if (aht20_write_cmd(&status, 1) != ERRCODE_SUCC) {
        osal_printk("AHT20: status cmd failed\r\n");
        return -1;
    }
    osal_msleep(50);

    if (aht20_read_data(&status, 1) != ERRCODE_SUCC) {
        osal_printk("AHT20: status read failed\r\n");
        return -1;
    }

    osal_printk("AHT20: init done, status=0x%02X\r\n", status);

    if ((status & AHT20_STATUS_CALIBRATED) == 0) {
        osal_printk("AHT20: not calibrated, retrying init\r\n");
        /* Try init again */
        if (aht20_write_cmd(init_cmd, 2) != ERRCODE_SUCC) {
            return -1;
        }
        osal_msleep(100);
    }

    return 0;
}

int aht20_read_temperature_humidity(int32_t *temp, int32_t *humi)
{
    uint8_t cmd[3] = {AHT20_CMD_MEASURE, 0x33, 0x00};
    uint8_t data[6] = {0};
    uint32_t temp_raw, humi_raw;
    int timeout;

    /* Step 1: Trigger measurement */
    if (aht20_write_cmd(cmd, 3) != ERRCODE_SUCC) {
        return -1;
    }

    /* Step 2: Wait for measurement - polling busy bit */
    timeout = 0;
    while (timeout < 500) {
        osal_msleep(5);
        timeout += 5;

        if (aht20_read_data(data, 1) == ERRCODE_SUCC) {
            if (!(data[0] & AHT20_STATUS_BUSY)) {
                break;
            }
        }
    }

    if (timeout >= 500) {
        osal_printk("AHT20: measurement timeout\r\n");
        return -1;
    }

    /* Step 3: Read full 6 bytes of data */
    if (aht20_read_data(data, 6) != ERRCODE_SUCC) {
        osal_printk("AHT20: data read failed\r\n");
        return -1;
    }

    /* Print raw data for debugging */
    osal_printk("AHT20 raw: %02X %02X %02X %02X %02X %02X\r\n",
                data[0], data[1], data[2], data[3], data[4], data[5]);

    /* Step 4: Check if all zeros (invalid) */
    if (data[1] == 0 && data[2] == 0 && data[3] == 0 && data[4] == 0 && data[5] == 0) {
        osal_printk("AHT20: all zero data\r\n");
        return -1;
    }

    /* Step 5: Parse 20-bit humidity and temperature values
     * Byte layout from datasheet:
     *  [0]=status [1]=H[19:12] [2]=H[11:4] [3]=H[3:0]|T[19:16] [4]=T[15:8] [5]=T[7:0]
     */
    humi_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
    temp_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];

    osal_printk("AHT20: humi_raw=%u temp_raw=%u\r\n", humi_raw, temp_raw);

    /* Step 6: Convert to physical values
     * RH(%) = S_RH / 2^20 * 100
     * T(°C) = S_T / 2^20 * 200 - 50
     *
     * Using intermediate float-like integer arithmetic:
     * First multiply by scale, then divide by 2^20 (1048576)
     */

    /* Humidity: value * 100 * 100 / 1048576 = value * 10000 / 1048576
     * Result is in centi-percent (e.g. 5025 = 50.25%) */
    *humi = (int32_t)(((uint64_t)humi_raw * 10000ULL) / 1048576ULL);

    /* Temperature: value * 200 * 100 / 1048576 - 5000 = value * 20000 / 1048576 - 5000
     * Result is in centi-degrees Celsius (e.g. 2250 = 22.50°C) */
    *temp = (int32_t)(((uint64_t)temp_raw * 20000ULL) / 1048576ULL) - 5000;

    osal_printk("AHT20: temp=%d.%02dC humi=%d.%02d%%\r\n",
                *temp / 100, (*temp > 0 ? *temp : -*temp) % 100,
                *humi / 100, *humi % 100);

    /* Step 7: Validate */
    if (*humi < 0 || *humi > 10000 || *temp < -5000 || *temp > 9000) {
        osal_printk("AHT20: value out of range: T=%d H=%d\r\n", *temp, *humi);
        return -1;
    }

    return 0;
}

int aht20_reset(void)
{
    uint8_t cmd = AHT20_CMD_RESET;
    if (aht20_write_cmd(&cmd, 1) != ERRCODE_SUCC) {
        return -1;
    }
    osal_msleep(20);
    return 0;
}

int aht20_is_calibrated(void)
{
    uint8_t status = AHT20_CMD_STATUS;
    uint8_t response;

    if (aht20_write_cmd(&status, 1) != ERRCODE_SUCC) {
        return -1;
    }
    osal_msleep(50);

    if (aht20_read_data(&response, 1) != ERRCODE_SUCC) {
        return -1;
    }

    return (response & AHT20_STATUS_CALIBRATED) ? 1 : 0;
}