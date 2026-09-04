/*
 * Environment Monitor for WS63 - AHT20 + MQ-2 + OLED
 */

#include "pinctrl.h"
#include "i2c.h"
#include "soc_osal.h"
#include "app_init.h"
#include "ssd1306.h"
#include "aht20.h"
#include "mq2.h"

#define I2C_SCL_PIN    15
#define I2C_SDA_PIN    16
#define I2C_PIN_MODE   2
#define I2C_BAUDRATE   400000

#define MQ2_ADC_CHANNEL  5     /* ADC channel for MQ-2 AOUT, change based on your board */
#define MQ2_PREHEAT_MS   30000 /* MQ-2 preheat time: 30 seconds */

#define TASK_PRIO        24
#define TASK_STACK_SIZE  0x2000
#define READ_INTERVAL_MS 1000

static void i2c_pin_init(void)
{
    uapi_pin_set_mode(I2C_SCL_PIN, I2C_PIN_MODE);
    uapi_pin_set_mode(I2C_SDA_PIN, I2C_PIN_MODE);
}

static void *env_monitor_task(const char *arg)
{
    unused(arg);
    int32_t temperature, humidity;
    uint16_t gas_voltage;
    mq2_level_t gas_level;
    char temp_str[24];
    char humi_str[24];
    char gas_str[24];
    int ret, temp_int, temp_frac, humi_int, humi_frac;

    /* ---- Init I2C ---- */
    i2c_pin_init();
    ret = uapi_i2c_master_init(1, I2C_BAUDRATE, 0);
    if (ret != ERRCODE_SUCC) {
        osal_printk("I2C init failed: 0x%x\r\n", ret);
        return NULL;
    }

    /* ---- Init OLED ---- */
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_DrawString("Env Monitor", Font_7x10, White);
    ssd1306_UpdateScreen();

    /* ---- Init AHT20 ---- */
    osal_msleep(100);
    ret = aht20_init();
    if (ret != 0) {
        osal_printk("AHT20 init failed\r\n");
        ssd1306_SetCursor(0, 20);
        ssd1306_DrawString("AHT20 Error", Font_7x10, White);
        ssd1306_UpdateScreen();
        return NULL;
    }

    /* ---- Init MQ-2 ---- */
    ret = mq2_init(MQ2_ADC_CHANNEL);
    if (ret != 0) {
        osal_printk("MQ2 init failed\r\n");
        ssd1306_SetCursor(0, 42);
        ssd1306_DrawString("MQ2 Error", Font_7x10, White);
        ssd1306_UpdateScreen();
        return NULL;
    }

    /* MQ-2 preheat */
    ssd1306_SetCursor(0, 42);
    ssd1306_DrawString("Gas:Warming...", Font_7x10, White);
    ssd1306_UpdateScreen();
    osal_msleep(MQ2_PREHEAT_MS);

    osal_printk("Env Monitor started\r\n");

    while (1) {
        /* ---- Read AHT20 ---- */
        ret = aht20_read_temperature_humidity(&temperature, &humidity);
        if (ret == 0) {
            temp_int = temperature / 100;
            temp_frac = (temperature >= 0) ? (temperature % 100) : ((-temperature) % 100);
            humi_int = humidity / 100;
            humi_frac = humidity % 100;

            /* Display temperature */
            ssd1306_SetCursor(0, 14);
            ssd1306_DrawString("                ", Font_7x10, White);
            snprintf(temp_str, sizeof(temp_str), "T:%d.%02dC", temp_int, temp_frac);
            ssd1306_SetCursor(0, 14);
            ssd1306_DrawString(temp_str, Font_7x10, White);

            /* Display humidity */
            ssd1306_SetCursor(0, 28);
            ssd1306_DrawString("                ", Font_7x10, White);
            snprintf(humi_str, sizeof(humi_str), "H:%d.%02d%%", humi_int, humi_frac);
            ssd1306_SetCursor(0, 28);
            ssd1306_DrawString(humi_str, Font_7x10, White);

            osal_printk("T:%d.%02dC H:%d.%02d%%\r\n",
                       temp_int, temp_frac, humi_int, humi_frac);
        }

        /* ---- Read MQ-2 ---- */
        ret = mq2_read_voltage(&gas_voltage);
        if (ret == 0) {
            gas_level = mq2_get_level(gas_voltage);

            /* Display gas info */
            ssd1306_SetCursor(0, 42);
            ssd1306_DrawString("                ", Font_7x10, White);
            snprintf(gas_str, sizeof(gas_str), "Gas:%s %dmV",
                     mq2_level_string(gas_level), gas_voltage);
            ssd1306_SetCursor(0, 42);
            ssd1306_DrawString(gas_str, Font_7x10, White);

            osal_printk("Gas: %s, %dmV\r\n",
                       mq2_level_string(gas_level), gas_voltage);
        }

        ssd1306_UpdateScreen();
        osal_msleep(READ_INTERVAL_MS);
    }

    return NULL;
}

static void env_monitor_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create(
        (osal_kthread_handler)env_monitor_task,
        0, "EnvMonitor", TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, TASK_PRIO);
        osal_kfree(task_handle);
    } else {
        osal_printk("Create task failed\r\n");
    }
    osal_kthread_unlock();
}

app_run(env_monitor_entry);