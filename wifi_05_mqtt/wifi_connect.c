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

#include "wifi_connect.h"
#include "errcode.h" 
#include "lwip/netifapi.h"
#include "wifi_hotspot.h"
#include "wifi_hotspot_config.h"
#include "stdlib.h"
#include "uart.h"
#include "lwip/nettool/misc.h"
#include "soc_osal.h"

#define WIFI_IFNAME_MAX_SIZE 16
#define WIFI_SCAN_AP_LIMIT 64
#define WIFI_CONN_STATUS_MAX_GET_TIMES 5
#define DHCP_BOUND_STATUS_MAX_GET_TIMES 20
#define WIFI_STA_IP_MAX_GET_TIMES 5
#define WIFI_RETRY_MAX_TIMES 3  /* 最大重试次数 */

static errcode_t get_match_network(const char *expected_ssid,
                                   const char *key,
                                   wifi_sta_config_stru *expected_bss)
{
    uint32_t num = WIFI_SCAN_AP_LIMIT;
    uint32_t bss_index = 0;
    uint32_t scan_len = sizeof(wifi_scan_info_stru) * WIFI_SCAN_AP_LIMIT;
    wifi_scan_info_stru *result = osal_kmalloc(scan_len, OSAL_GFP_ATOMIC);
    if (result == NULL) {
        return ERRCODE_MALLOC;
    }

    memset_s(result, scan_len, 0, scan_len);
    if (wifi_sta_get_scan_info(result, &num) != ERRCODE_SUCC) {
        osal_kfree(result);
        return ERRCODE_FAIL;
    }

    for (bss_index = 0; bss_index < num; bss_index++) {
        if (strlen(expected_ssid) == strlen(result[bss_index].ssid)) {
            if (memcmp(expected_ssid, result[bss_index].ssid, strlen(expected_ssid)) == 0) {
                break;
            }
        }
    }

    if (bss_index >= num) {
        osal_kfree(result);
        return ERRCODE_FAIL;
    }

    if (memcpy_s(expected_bss->ssid, WIFI_MAX_SSID_LEN,
                 result[bss_index].ssid, WIFI_MAX_SSID_LEN) != EOK) {
        osal_kfree(result);
        return ERRCODE_MEMCPY;
    }
    if (memcpy_s(expected_bss->bssid, WIFI_MAC_LEN,
                 result[bss_index].bssid, WIFI_MAC_LEN) != EOK) {
        osal_kfree(result);
        return ERRCODE_MEMCPY;
    }
    expected_bss->security_type = result[bss_index].security_type;
    if (memcpy_s(expected_bss->pre_shared_key, WIFI_MAX_KEY_LEN, key, strlen(key)) != EOK) {
        osal_kfree(result);
        return ERRCODE_MEMCPY;
    }
    expected_bss->ip_type = DHCP;
    osal_kfree(result);
    return ERRCODE_SUCC;
}

static errcode_t wifi_connect_once(void)
{
    char ifname[WIFI_IFNAME_MAX_SIZE + 1] = "wlan0";
    wifi_sta_config_stru expected_bss = {0};
    struct netif *netif_p = NULL;
    wifi_linked_info_stru wifi_status;
    uint8_t index = 0;

    if (wifi_sta_enable() != ERRCODE_SUCC) {
        printf("sta enable fail !\r\n");
        return ERRCODE_FAIL;
    }

    do {
        printf("Start Scan !\r\n");
        osal_msleep(1000);
        if (wifi_sta_scan() != ERRCODE_SUCC) {
            printf("STA scan fail, try again !\r\n");
            continue;
        }

        osal_msleep(3000);

        if (get_match_network(CONFIG_WIFI_SSID, CONFIG_WIFI_PWD, &expected_bss) != ERRCODE_SUCC) {
            printf("Can not find AP, try again !\r\n");
            continue;
        }

        printf("STA try connect.\r\n");
        if (wifi_sta_connect(&expected_bss) != ERRCODE_SUCC) {
            continue;
        }

        for (index = 0; index < WIFI_CONN_STATUS_MAX_GET_TIMES; index++) {
            osal_msleep(500);
            memset_s(&wifi_status, sizeof(wifi_linked_info_stru), 0, sizeof(wifi_linked_info_stru));
            if (wifi_sta_get_ap_info(&wifi_status) != ERRCODE_SUCC) {
                continue;
            }
            if (wifi_status.conn_state == WIFI_CONNECTED) {
                break;
            }
        }

        if (wifi_status.conn_state == WIFI_CONNECTED) {
            break;
        }
    } while (1);

    printf("STA DHCP start.\r\n");
    netif_p = netifapi_netif_find(ifname);
    if (netif_p == NULL) {
        return ERRCODE_FAIL;
    }

    if (netifapi_dhcp_start(netif_p) != ERR_OK) {
        printf("STA DHCP Fail.\r\n");
        return ERRCODE_FAIL;
    }

    for (uint8_t i = 0; i < DHCP_BOUND_STATUS_MAX_GET_TIMES; i++) {
        osal_msleep(500);
        if (netifapi_dhcp_is_bound(netif_p) == ERR_OK) {
            printf("STA DHCP bound success.\r\n");
            break;
        }
    }

    for (uint8_t i = 0; i < WIFI_STA_IP_MAX_GET_TIMES; i++) {
        osal_msleep(10);
        if (netif_p->ip_addr.u_addr.ip4.addr != 0) {
            printf("STA IP %u.%u.%u.%u\r\n",
                   (netif_p->ip_addr.u_addr.ip4.addr & 0x000000ff),
                   (netif_p->ip_addr.u_addr.ip4.addr & 0x0000ff00) >> 8,
                   (netif_p->ip_addr.u_addr.ip4.addr & 0x00ff0000) >> 16,
                   (netif_p->ip_addr.u_addr.ip4.addr & 0xff000000) >> 24);
            printf("STA connect success.\r\n");
            return ERRCODE_SUCC;
        }
    }

    printf("STA connect fail.\r\n");
    return ERRCODE_FAIL;
}

errcode_t wifi_connect(void)
{
    errcode_t ret;
    uint8_t retry_count = 0;

    /* 等待WiFi初始化完成 */
    (void)osal_msleep(5000);
    printf("WiFi: Starting connection, max retry %d times...\r\n", WIFI_RETRY_MAX_TIMES);

    for (retry_count = 0; retry_count < WIFI_RETRY_MAX_TIMES; retry_count++) {
        printf("WiFi: Connection attempt %d/%d\r\n", retry_count + 1, WIFI_RETRY_MAX_TIMES);
        
        ret = wifi_connect_once();
        if (ret == ERRCODE_SUCC) {
            printf("WiFi: Connect success!\r\n");
            return ERRCODE_SUCC;
        }

        /* 如果不是最后一次尝试，等待一段时间再重试 */
        if (retry_count < WIFI_RETRY_MAX_TIMES - 1) {
            printf("WiFi: Retrying after 3 seconds...\r\n");
            osal_msleep(3000);
        }
    }

    printf("WiFi: All %d attempts failed!\r\n", WIFI_RETRY_MAX_TIMES);
    return ERRCODE_FAIL;
}