#include "soc_osal.h"
#include "app_init.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../../open_source/mqtt/paho.mqtt.c/src/MQTTClientPersistence.h"
#include "../../../open_source/mqtt/paho.mqtt.c/src/MQTTClient.h"
#include "errcode.h"
#include "wifi_connect.h"
#include "bsp.h"

osThreadId_t mqtt_init_task_id; // mqtt订阅数据任务

#define SERVER_IP_ADDR "tcp://a413ff5de7.st1.iotda-device.cn-north-4.myhuaweicloud.com" // 接入地址
#define SERVER_IP_PORT 1883                                                       // 端口号
#define CLIENT_ID "6a5b1b8dcbb0cf6bb9704ce2_hi3863_0_0_2026080507"                 // 设备id（更新为最新生成）

#define MQTT_CMDTOPIC_SUB "$oc/devices/6a5b1b8dcbb0cf6bb9704ce2_hi3863/sys/commands/set/#" // 平台下发命令

#define MQTT_DATATOPIC_PUB "$oc/devices/6a5b1b8dcbb0cf6bb9704ce2_hi3863/sys/properties/report"              // 属性上报topic
#define MQTT_CLIENT_RESPONSE "$oc/devices/6a5b1b8dcbb0cf6bb9704ce2_hi3863/sys/commands/response/request_id=%s" // 命令响应topic

#define DATA_SEVER_NAME "Switch"
#define DATA_ATTR_NAME "beep_stat"
#define MQTT_DATA_SEND "{\"services\": [{\"service_id\": \"%s\",\"properties\": {\"%s\": %s }}]}" // 上报数据格式

#define KEEP_ALIVE_INTERVAL 120
#define DELAY_TIME_MS 200
#define IOT

#ifdef IOT
char *g_username = "6a5b1b8dcbb0cf6bb9704ce2_hi3863";
char *g_password = "5d3dae22de08aef6889c513ab829f9b6935ed79cded391d2b26532ae86f535b0"; // 更新为最新生成
#endif

char g_send_buffer[512] = {0};   // 发布数据缓冲区
char g_response_id[100] = {0};  // 保存命令id缓冲区
char g_response_buf[] =
    "{\"result_code\": 0,\"response_name\": \"beep\",\"paras\": {\"result\": \"success\"}}"; // 响应json
uint8_t g_cmdFlag = 0;
MQTTClient client;
volatile MQTTClient_deliveryToken deliveredToken;
extern int MQTTClient_init(void);

/* 回调函数，处理连接丢失 */
void connlost(void *context, char *cause)
{
    unused(context);
    printf("Connection lost: %s\n", cause);
}

int mqtt_subscribe(const char *topic)
{
    printf("subscribe start\r\n");
    MQTTClient_subscribe(client, topic, 1);
    return 0;
}

int mqtt_publish(const char *topic, char *msg)
{
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int ret = 0;
    pubmsg.payload = msg;
    pubmsg.payloadlen = (int)strlen(msg);
    pubmsg.qos = 1;
    pubmsg.retained = 0;
    printf("[payload]:  %s, [topic]: %s\r\n", msg, topic);
    ret = MQTTClient_publishMessage(client, topic, &pubmsg, &token);
    if (ret != MQTTCLIENT_SUCCESS) {
        printf("mqtt publish failed\r\n");
        return ret;
    }
    return ret;
}

/* 回调函数，处理消息到达 */
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    unused(context);
    printf("Message with token value %d delivery confirmed\n", dt);
    deliveredToken = dt;
}

// 解析字符串并保存到数组中
void parse_after_equal(const char *input, char *output)
{
    const char *equalsign = strchr(input, '=');
    if (equalsign != NULL) {
        strcpy(output, equalsign + 1);
    }
}

/* 回调函数，处理接收到的消息 */
int messageArrived(void *context, char *topic_name, int topic_len, MQTTClient_message *message)
{
    unused(context);
    unused(topic_len);
    printf("[Message recv topic]: %s\n", topic_name);
    printf("[Message]: %s\n", (char *)message->payload);

    // 进行传感器控制
    if (strstr((char *)message->payload, "true") != NULL)
        my_io_setval(SENSOR_IO, GPIO_LEVEL_HIGH);
    else
        my_io_setval(SENSOR_IO, GPIO_LEVEL_LOW);

    // 解析命令id
    parse_after_equal(topic_name, g_response_id);
    g_cmdFlag = 1;
    memset((char *)message->payload, 0, message->payloadlen);

    return 1; // 表示消息已被处理
}

static errcode_t mqtt_connect(void)
{
    int ret;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    /* 初始化MQTT客户端 */
    MQTTClient_init();

    /* 创建 MQTT 客户端 */
    ret = MQTTClient_create(&client, SERVER_IP_ADDR, CLIENT_ID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    if (ret != MQTTCLIENT_SUCCESS) {
        printf("Failed to create MQTT client, return code %d\n", ret);
        return ERRCODE_FAIL;
    }

    conn_opts.keepAliveInterval = KEEP_ALIVE_INTERVAL;
    conn_opts.cleansession = 1;

#ifdef IOT
    conn_opts.username = g_username;
    conn_opts.password = g_password;
#endif

    // 绑定回调函数
    MQTTClient_setCallbacks(client, NULL, connlost, messageArrived, delivered);

    // 尝试连接
    if ((ret = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, return code %d\n", ret);
        MQTTClient_destroy(&client); // 连接失败时销毁客户端
        return ERRCODE_FAIL;
    }

    printf("Connected to MQTT broker!\n");
    osDelay(DELAY_TIME_MS);

    // 订阅MQTT主题
    mqtt_subscribe(MQTT_CMDTOPIC_SUB);

    while (1) {
        // 响应平台命令部分
        osDelay(DELAY_TIME_MS);
        if (g_cmdFlag) {
            sprintf(g_send_buffer, MQTT_CLIENT_RESPONSE, g_response_id);
            mqtt_publish(g_send_buffer, g_response_buf);
            g_cmdFlag = 0;
            memset(g_response_id, 0, sizeof(g_response_id));
        }

        // 属性上报部分
        osDelay(DELAY_TIME_MS);
        memset(g_send_buffer, 0, sizeof(g_send_buffer));
        sprintf(g_send_buffer, MQTT_DATA_SEND, DATA_SEVER_NAME, DATA_ATTR_NAME,
                my_io_readval(SENSOR_IO) ? "true" : "false");
        mqtt_publish(MQTT_DATATOPIC_PUB, g_send_buffer);
        memset(g_send_buffer, 0, sizeof(g_send_buffer));
    }

    return ERRCODE_SUCC;
}

void mqtt_init_task(const char *argument)
{
    unused(argument);
    my_gpio_init(SENSOR_IO);
    wifi_connect();
    osDelay(DELAY_TIME_MS);
    mqtt_connect();
}

static void network_wifi_mqtt_example(void)
{
    printf("Enter HUAWEI IOT example()!\n");

    osThreadAttr_t options = {0};
    options.name = "mqtt_init_task";
    options.attr_bits = 0;
    options.cb_mem = NULL;
    options.cb_size = 0;
    options.stack_mem = NULL;
    options.stack_size = 0x2000;
    options.priority = osPriorityNormal;

    mqtt_init_task_id = osThreadNew((osThreadFunc_t)mqtt_init_task, NULL, &options);
    if (mqtt_init_task_id != NULL) {
        printf("ID = %p, Create mqtt_init_task_id is OK!\n", (void*)mqtt_init_task_id);
    }
}

/* Run the sample. */
app_run(network_wifi_mqtt_example);