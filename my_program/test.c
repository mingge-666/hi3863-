/*
 * ============================================================
 *  华为云 IoT 智能蜂鸣器远程控制示例代码
 *  功能：通过 WiFi + MQTT 连接华为云 IoTDA 平台，
 *        实现远程控制蜂鸣器开关，并定时上报状态。
 *  硬件：Hi3863 开发板 + 蜂鸣器模块（接 SENSOR_IO 引脚）
 * ============================================================
 */

// ---------- 系统头文件 ----------
#include "soc_osal.h"      // 系统 OS 抽象层，提供任务、延时等接口
#include "app_init.h"      // 应用初始化框架，提供 app_run 宏
#include "cmsis_os2.h"     // CMSIS-RTOS v2 实时操作系统 API
#include <stdio.h>         // 标准输入输出，用于 printf 打印日志
#include <stdlib.h>        // 标准库，提供内存分配等功能
#include <string.h>        // 字符串操作，如 memset、strstr、strcpy

// ---------- MQTT 客户端库 ----------
#include "../../../open_source/mqtt/paho.mqtt.c/src/MQTTClientPersistence.h" // MQTT 持久化接口
#include "../../../open_source/mqtt/paho.mqtt.c/src/MQTTClient.h"             // Paho MQTT 异步客户端核心库

// ---------- 板级支持包 ----------
#include "errcode.h"       // 错误码定义
#include "wifi_connect.h"  // WiFi 连接封装函数（扫描、连接、DHCP）
#include "bsp.h"           // 板级支持包，提供 GPIO 控制函数（my_gpio_init、my_io_setval、my_io_readval）

// ---------- 全局变量 ----------
osThreadId_t mqtt_init_task_id; // MQTT 初始化任务的线程句柄，用于管理任务生命周期

// ============================================================
//  第一部分：华为云 IoTDA 连接参数配置
//  这些值需要从华为云控制台获取，不可随意更改
// ============================================================

// 华为云 IoTDA 设备接入地址（TCP 协议，非加密）
// 格式：tcp://<接入点>.st1.iotda-device.<区域>.myhuaweicloud.com
// 注意：区域必须与创建设备的区域一致，这里是 cn-north-4（华北-北京四）
#define SERVER_IP_ADDR "tcp://a413ff5de7.st1.iotda-device.cn-north-4.myhuaweicloud.com"

// MQTT 标准端口，非加密连接使用 1883
// 如果使用加密连接（TLS），端口应为 8883
#define SERVER_IP_PORT 1883

// MQTT 客户端标识符（Client ID）
// 格式：<设备ID>_0_0_<时间戳>
// 由华为云 MQTT 连接参数生成器生成，必须唯一
#define CLIENT_ID "6a5b1b8dcbb0cf6bb9704ce2_hi3863_0_1_2026071807"

// ============================================================
//  第二部分：MQTT 主题（Topic）定义
//  华为云 IoTDA 的设备 Topic 格式固定为：
//  $oc/devices/{device_id}/sys/{功能分类}/{具体操作}
// ============================================================

// 订阅主题：接收平台下发的命令
// 通配符 # 表示匹配所有子主题，可接收所有命令
#define MQTT_CMDTOPIC_SUB "$oc/devices/6a5b1b8dcbb0cf6bb9704ce2_hi3863/sys/commands/set/#"

// 发布主题：上报设备属性到平台
// 平台可通过此 Topic 获取设备当前状态
#define MQTT_DATATOPIC_PUB "$oc/devices/6a5b1b8dcbb0cf6bb9704ce2_hi3863/sys/properties/report"

// 发布主题：响应平台下发的命令（模板）
// %s 会被替换为具体的 request_id，用于关联命令和响应
#define MQTT_CLIENT_RESPONSE "$oc/devices/6a5b1b8dcbb0cf6bb9704ce2_hi3863/sys/commands/response/request_id=%s"

// ============================================================
//  第三部分：设备模型与服务定义
//  需与华为云控制台创建的产品模型完全一致
// ============================================================

#define DATA_SEVER_NAME "Switch"      // 服务名称：开关控制服务
#define DATA_ATTR_NAME "beep_stat"    // 属性名称：蜂鸣器状态（布尔值）

// 属性上报的数据格式（JSON 模板）
// %s 依次替换为：服务名称、属性名称、属性值（true/false）
#define MQTT_DATA_SEND "{\"services\": [{\"service_id\": \"%s\",\"properties\": {\"%s\": %s }}]}"

// ============================================================
//  第四部分：MQTT 连接参数
// ============================================================

#define KEEP_ALIVE_INTERVAL 120  // MQTT 心跳保活间隔（秒），超过此时间无通信则断开
#define DELAY_TIME_MS 200        // 通用延时时间（毫秒），用于任务调度让步

#define IOT  // 使能 IoT 连接参数宏，控制是否启用用户名密码认证

#ifdef IOT
// MQTT 用户名：即设备 ID（DeviceId）
char *g_username = "6a5b1b8dcbb0cf6bb9704ce2_hi3863";

// MQTT 密码：由华为云 MQTT 参数生成器基于 DeviceSecret 和时间戳计算得出
// 注意：这不是原始的 DeviceSecret，而是 HMAC-SHA256 加密后的结果
char *g_password = "8f409571638582bc01f99b891de04207067aa3c4aa0d1fe749b942fc48739108";
#endif

// ============================================================
//  第五部分：全局工作缓冲区
// ============================================================

char g_send_buffer[512] = {0};    // 通用发送缓冲区，用于拼接 JSON 数据
char g_response_id[100] = {0};    // 命令响应 ID 缓冲区，保存从 Topic 中解析出的 request_id

// 命令响应 JSON 模板（固定内容）
// result_code: 0 表示成功
// response_name: 响应命令名称
// paras.result: 执行结果描述
char g_response_buf[] =
    "{\"result_code\": 0,\"response_name\": \"beep\",\"paras\": {\"result\": \"success\"}}";

uint8_t g_cmdFlag = 0;  // 命令待响应标志位：1 表示有待响应的命令

MQTTClient client;                          // MQTT 客户端实例句柄
volatile MQTTClient_deliveryToken deliveredToken; // 最近一次已送达的消息令牌（volatile 防止编译器优化）

extern int MQTTClient_init(void);  // 外部函数声明：MQTT 客户端库初始化

// ============================================================
//  第六部分：MQTT 回调函数
// ============================================================

/**
 * @brief 连接断开回调函数
 * @param context 用户上下文指针（本例未使用）
 * @param cause   断开原因描述字符串
 * @note  当 MQTT 连接意外断开时，此函数被自动调用
 */
void connlost(void *context, char *cause)
{
    unused(context);  // 消除未使用参数的编译警告
    printf("Connection lost: %s\n", cause);  // 打印断开原因，便于调试
}

/**
 * @brief 订阅指定 MQTT 主题
 * @param topic 要订阅的主题字符串
 * @return 0 表示成功
 * @note  使用 QoS 1（至少一次），确保消息可靠传递
 */
int mqtt_subscribe(const char *topic)
{
    printf("subscribe start\r\n");
    MQTTClient_subscribe(client, topic, 1);  // 第二个参数 1 表示 QoS 等级
    return 0;
}

/**
 * @brief 向指定主题发布 MQTT 消息
 * @param topic 目标主题
 * @param msg   要发送的消息内容（字符串）
 * @return MQTTCLIENT_SUCCESS 表示成功，其他值表示失败
 * @note  使用 QoS 1，消息会重试直到收到 PUBACK 确认
 */
int mqtt_publish(const char *topic, char *msg)
{
    MQTTClient_message pubmsg = MQTTClient_message_initializer; // 初始化消息结构体
    MQTTClient_deliveryToken token;  // 用于跟踪消息投递状态的令牌
    int ret = 0;

    // 填充消息内容
    pubmsg.payload = msg;                    // 消息体指针
    pubmsg.payloadlen = (int)strlen(msg);    // 消息长度
    pubmsg.qos = 1;                         // 服务质量：至少一次
    pubmsg.retained = 0;                    // 不保留消息（新订阅者不会收到旧消息）

    printf("[payload]:  %s, [topic]: %s\r\n", msg, topic);

    // 发布消息，token 会返回唯一的消息 ID
    ret = MQTTClient_publishMessage(client, topic, &pubmsg, &token);
    if (ret != MQTTCLIENT_SUCCESS) {
        printf("mqtt publish failed\r\n");
        return ret;
    }
    return ret;
}

/**
 * @brief 消息送达确认回调函数
 * @param context 用户上下文
 * @param dt      已送达消息的令牌（token）
 * @note  当 QoS 1 或 QoS 2 的消息被平台确认接收后调用
 */
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    unused(context);
    printf("Message with token value %d delivery confirmed\n", dt);
    deliveredToken = dt;  // 记录最后一次确认的令牌
}

/**
 * @brief 从字符串中提取等号后面的内容
 * @param input  输入字符串（如 "request_id=12345"）
 * @param output 输出缓冲区，存储提取后的内容（如 "12345"）
 * @note  用于从命令响应 Topic 中解析出 request_id
 */
void parse_after_equal(const char *input, char *output)
{
    const char *equalsign = strchr(input, '=');  // 查找等号位置
    if (equalsign != NULL) {
        strcpy(output, equalsign + 1);  // 复制等号后面的所有字符
    }
}

/**
 * @brief 消息到达回调函数（核心业务逻辑）
 * @param context    用户上下文
 * @param topic_name 消息来源主题
 * @param topic_len  主题长度
 * @param message    接收到的消息内容
 * @return 1 表示消息已被处理，0 表示未处理
 * @note  当平台下发命令时，此函数被自动调用
 *
 * 华为云下发命令的 JSON 格式示例：
 * {
 *   "object_device_id": "6a5b1b8dcbb0cf6bb9704ce2_hi3863",
 *   "command_name": "beep_control",
 *   "service_id": "Switch",
 *   "paras": {
 *     "beep_stat": true
 *   }
 * }
 */
int messageArrived(void *context, char *topic_name, int topic_len, MQTTClient_message *message)
{
    unused(context);
    unused(topic_len);

    printf("[Message recv topic]: %s\n", topic_name);
    printf("[Message]: %s\n", (char *)message->payload);

    // ---- 核心控制逻辑 ----
    // 检查消息体中是否包含 "true" 字符串
    // 如果包含，说明平台要求打开蜂鸣器
    // 否则关闭蜂鸣器
    if (strstr((char *)message->payload, "true") != NULL)
        my_io_setval(SENSOR_IO, GPIO_LEVEL_HIGH);  // 设置 GPIO 为高电平，蜂鸣器响
    else
        my_io_setval(SENSOR_IO, GPIO_LEVEL_LOW);   // 设置 GPIO 为低电平，蜂鸣器静音

    // ---- 准备命令响应 ----
    // 从 Topic 中提取 request_id（格式：.../request_id=xxxxx）
    parse_after_equal(topic_name, g_response_id);

    // 设置命令待响应标志，主循环会检测此标志并发送响应
    g_cmdFlag = 1;

    // 清理消息负载（防止重复处理）
    memset((char *)message->payload, 0, message->payloadlen);

    return 1; // 返回 1 表示消息已被处理
}

// ============================================================
//  第七部分：MQTT 连接与主循环
// ============================================================

/**
 * @brief MQTT 连接与数据处理主函数
 * @return ERRCODE_SUCC 表示成功，ERRCODE_FAIL 表示失败
 * @note  此函数会阻塞运行，内部包含无限循环
 *
 * 执行流程：
 * 1. 初始化 MQTT 客户端库
 * 2. 创建 MQTT 客户端实例
 * 3. 设置连接参数（服务器地址、用户名、密码）
 * 4. 绑定回调函数
 * 5. 发起连接
 * 6. 订阅命令 Topic
 * 7. 进入无限循环：处理命令响应 + 定时上报属性
 */
static errcode_t mqtt_connect(void)
{
    int ret;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    /* 第一步：初始化 MQTT 客户端库 */
    MQTTClient_init();

    /* 第二步：创建 MQTT 客户端实例 */
    // 参数：客户端句柄、服务器地址、客户端 ID、持久化方式、持久化上下文
    ret = MQTTClient_create(&client, SERVER_IP_ADDR, CLIENT_ID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    if (ret != MQTTCLIENT_SUCCESS) {
        printf("Failed to create MQTT client, return code %d\n", ret);
        return ERRCODE_FAIL;
    }

    /* 第三步：配置连接选项 */
    conn_opts.keepAliveInterval = KEEP_ALIVE_INTERVAL;  // 心跳间隔
    conn_opts.cleansession = 1;                        // 清除会话（每次连接都是全新开始）

#ifdef IOT
    // 设置 MQTT 用户名和密码（华为云 IoTDA 要求）
    conn_opts.username = g_username;
    conn_opts.password = g_password;
#endif

    /* 第四步：绑定回调函数 */
    // 参数：客户端句柄、用户上下文、连接断开回调、消息到达回调、送达确认回调
    MQTTClient_setCallbacks(client, NULL, connlost, messageArrived, delivered);

    /* 第五步：发起 MQTT 连接 */
    if ((ret = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, return code %d\n", ret);
        MQTTClient_destroy(&client); // 连接失败时销毁客户端，释放资源
        return ERRCODE_FAIL;
    }

    printf("Connected to MQTT broker!\n");
    osDelay(DELAY_TIME_MS);

    /* 第六步：订阅命令下发主题 */
    mqtt_subscribe(MQTT_CMDTOPIC_SUB);

    /* 第七步：进入主循环（永不退出） */
    while (1) {

        // ---- 任务 A：处理命令响应 ----
        osDelay(DELAY_TIME_MS);  // 短暂延时，让出 CPU 给其他任务
        if (g_cmdFlag) {
            // 构建完整的命令响应 Topic
            sprintf(g_send_buffer, MQTT_CLIENT_RESPONSE, g_response_id);
            
            // 发布命令响应消息
            mqtt_publish(g_send_buffer, g_response_buf);
            
            // 清除标志位和缓冲区，准备处理下一条命令
            g_cmdFlag = 0;
            memset(g_response_id, 0, sizeof(g_response_id));
        }

        // ---- 任务 B：定时上报设备属性 ----
        osDelay(DELAY_TIME_MS);
        
        // 清空发送缓冲区
        memset(g_send_buffer, 0, sizeof(g_send_buffer));
        
        // 构建属性上报 JSON
        // 读取当前蜂鸣器引脚的电平状态，转换为 "true" 或 "false"
        sprintf(g_send_buffer, MQTT_DATA_SEND, DATA_SEVER_NAME, DATA_ATTR_NAME,
                my_io_readval(SENSOR_IO) ? "true" : "false");
        
        // 发布属性数据到平台
        mqtt_publish(MQTT_DATATOPIC_PUB, g_send_buffer);
        
        // 再次清空缓冲区
        memset(g_send_buffer, 0, sizeof(g_send_buffer));
    }

    return ERRCODE_SUCC;  // 理论上永远不会执行到这里
}

// ============================================================
//  第八部分：任务入口函数
// ============================================================

/**
 * @brief MQTT 初始化任务入口
 * @param argument 任务参数（本例未使用）
 * @note  此函数作为 RTOS 任务的入口点
 *
 * 执行顺序：
 * 1. 初始化 GPIO（设置蜂鸣器引脚为输出模式）
 * 2. 连接 WiFi（阻塞等待连接成功）
 * 3. 短暂延时等待网络稳定
 * 4. 启动 MQTT 连接（进入无限循环）
 */
void mqtt_init_task(const char *argument)
{
    unused(argument);
    
    my_gpio_init(SENSOR_IO);  // 初始化蜂鸣器控制的 GPIO 引脚
    wifi_connect();            // 连接 WiFi（函数内部会等待连接成功）
    osDelay(DELAY_TIME_MS);   // 等待网络完全就绪
    
    mqtt_connect();           // 启动 MQTT 连接（此函数不会返回）
}

// ============================================================
//  第九部分：应用程序入口
// ============================================================

/**
 * @brief 网络 WiFi MQTT 示例主函数
 * @note  此函数由 app_run 宏调度执行
 *
 * 功能：
 * 1. 创建一个 RTOS 线程
 * 2. 在线程中执行 MQTT 初始化任务
 * 3. 线程栈大小为 0x2000（8KB），优先级为普通
 */
static void network_wifi_mqtt_example(void)
{
    printf("Enter HUAWEI IOT example()!\n");

    // 配置线程属性
    osThreadAttr_t options = {0};
    options.name = "mqtt_init_task";     // 线程名称（便于调试）
    options.attr_bits = 0;               // 属性位（默认）
    options.cb_mem = NULL;               // 控制块内存（动态分配）
    options.cb_size = 0;                 // 控制块大小（动态分配）
    options.stack_mem = NULL;            // 栈内存（动态分配）
    options.stack_size = 0x2000;         // 栈大小：8KB（根据实际需求调整）
    options.priority = osPriorityNormal; // 线程优先级：普通

    // 创建并启动 MQTT 初始化线程
    mqtt_init_task_id = osThreadNew((osThreadFunc_t)mqtt_init_task, NULL, &options);
    if (mqtt_init_task_id != NULL) {
        printf("ID = %p, Create mqtt_init_task_id is OK!\n", (void*)mqtt_init_task_id);
    }
}

/* 使用 app_run 宏注册应用程序入口 */
// app_run 是华为 SDK 提供的应用初始化宏
// 会在系统启动后自动调用 network_wifi_mqtt_example 函数
app_run(network_wifi_mqtt_example);