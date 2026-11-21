#include "heater_control.h"
#include "web_server.h"
#include "wifi_manager.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "heatbench";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(heater_control_init());
    ESP_ERROR_CHECK(wifi_manager_start());
    ESP_ERROR_CHECK(web_server_start());

    ESP_LOGI(TAG, "HeatBench started. SSR remains off until a valid sensor sample and an explicit command.");
}
