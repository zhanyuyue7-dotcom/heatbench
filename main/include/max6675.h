#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float temperature_c;
    bool thermocouple_open;
    uint16_t raw;
} max6675_sample_t;

esp_err_t max6675_init(void);
esp_err_t max6675_read(max6675_sample_t *sample);
