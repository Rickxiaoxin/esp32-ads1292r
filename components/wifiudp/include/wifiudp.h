#ifndef WIFIUDP_H_
#define WIFIUDP_H_

#include "esp_event.h"

#define WIFI_SSID "WeiShx"
#define WIFI_PASS "19980510"
#define HOST_IP_ADDR "192.168.1.173"
#define PORT 3333
#define WIFI_MAXIMUM_RETRY 10
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#define H2E_IDENTIFIER ""

void udp_client_task(void *pvParameters);
void event_handler(void *arg, esp_event_base_t event_base,
                   int32_t event_id, void *event_data);
void wifi_init_sta(void);

#endif
