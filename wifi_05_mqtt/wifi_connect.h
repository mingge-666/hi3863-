#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include "errcode.h" 

#define CONFIG_WIFI_SSID "写入你的WiFi名"
#define CONFIG_WIFI_PWD "你的wifi密码"

errcode_t wifi_connect(void);
#endif
