#include "wifi_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

static const char *TAG = "wifi";

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Station disconnected; retrying while local AP remains available");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        ESP_LOGI(TAG, "Station address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

esp_err_t wifi_manager_start(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop init failed");

    esp_netif_create_default_wifi_ap();
    const bool station_enabled = strlen(CONFIG_HEATER_STA_SSID) > 0;
    if (station_enabled) {
        esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "esp_wifi_init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                    wifi_event_handler, NULL),
                        TAG, "Wi-Fi event registration failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                    wifi_event_handler, NULL),
                        TAG, "IP event registration failed");

    uint8_t mac[6] = {0};
    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP), TAG, "MAC read failed");

    char ap_ssid[33];
    snprintf(ap_ssid, sizeof(ap_ssid), "HeatBench-%02X%02X", mac[4], mac[5]);

    wifi_config_t ap_config = {0};
    strlcpy((char *)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = strlen(ap_ssid);
    strlcpy((char *)ap_config.ap.password, CONFIG_HEATER_AP_PASSWORD,
            sizeof(ap_config.ap.password));
    ap_config.ap.channel = 6;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = strlen(CONFIG_HEATER_AP_PASSWORD) >= 8
                                ? WIFI_AUTH_WPA2_PSK
                                : WIFI_AUTH_OPEN;
    ap_config.ap.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(station_enabled ? WIFI_MODE_APSTA : WIFI_MODE_AP),
                        TAG, "Wi-Fi mode setup failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config), TAG,
                        "AP configuration failed");

    if (station_enabled) {
        wifi_config_t sta_config = {0};
        strlcpy((char *)sta_config.sta.ssid, CONFIG_HEATER_STA_SSID,
                sizeof(sta_config.sta.ssid));
        strlcpy((char *)sta_config.sta.password, CONFIG_HEATER_STA_PASSWORD,
                sizeof(sta_config.sta.password));
        // Allow an explicitly configured open network as well as secured networks.
        sta_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        sta_config.sta.pmf_cfg.capable = true;
        sta_config.sta.pmf_cfg.required = false;
        ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_config), TAG,
                            "station configuration failed");
    }

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");
    ESP_LOGI(TAG, "Local control AP: %s, dashboard: http://192.168.4.1", ap_ssid);
    return ESP_OK;
}
