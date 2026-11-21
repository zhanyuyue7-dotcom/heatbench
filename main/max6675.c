#include "max6675.h"

#include <string.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "max6675";
static spi_device_handle_t s_device;

esp_err_t max6675_init(void)
{
    const spi_bus_config_t bus_config = {
        .mosi_io_num = -1,
        .miso_io_num = CONFIG_HEATER_MAX6675_SO_GPIO,
        .sclk_io_num = CONFIG_HEATER_MAX6675_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };
    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = 1000 * 1000,
        .mode = 0,
        .spics_io_num = CONFIG_HEATER_MAX6675_CS_GPIO,
        .queue_size = 1,
    };

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_DISABLED), TAG,
                        "SPI bus initialization failed");
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &device_config, &s_device), TAG,
                        "MAX6675 device registration failed");

    ESP_LOGI(TAG, "MAX6675 ready: SCK=%d CS=%d SO=%d, sample period >=250 ms",
             CONFIG_HEATER_MAX6675_SCK_GPIO,
             CONFIG_HEATER_MAX6675_CS_GPIO,
             CONFIG_HEATER_MAX6675_SO_GPIO);
    return ESP_OK;
}

esp_err_t max6675_read(max6675_sample_t *sample)
{
    ESP_RETURN_ON_FALSE(sample != NULL, ESP_ERR_INVALID_ARG, TAG, "sample is NULL");

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.length = 16;
    transaction.flags = SPI_TRANS_USE_RXDATA;

    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(s_device, &transaction), TAG,
                        "SPI transaction failed");

    const uint16_t raw = ((uint16_t)transaction.rx_data[0] << 8) | transaction.rx_data[1];
    sample->raw = raw;
    sample->thermocouple_open = (raw & (1U << 2)) != 0;
    sample->temperature_c = (float)((raw >> 3) & 0x0FFFU) * 0.25f;
    return ESP_OK;
}
