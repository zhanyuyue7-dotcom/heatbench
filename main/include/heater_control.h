#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    HEATER_MODE_IDLE = 0,
    HEATER_MODE_CONSTANT,
    HEATER_MODE_PREHEAT,
    HEATER_MODE_BOOST,
    HEATER_MODE_COOLDOWN,
    HEATER_MODE_CALIBRATE_HEAT,
    HEATER_MODE_CALIBRATE_COAST,
    HEATER_MODE_FAULT,
} heater_mode_t;

typedef enum {
    HEATER_ALGORITHM_STAGED = 0,
    HEATER_ALGORITHM_PID,
} heater_algorithm_t;

typedef enum {
    HEATER_FAULT_NONE = 0,
    HEATER_FAULT_THERMOCOUPLE_OPEN,
    HEATER_FAULT_OVERTEMPERATURE,
    HEATER_FAULT_USER_ESTOP,
    HEATER_FAULT_INTERNAL,
} heater_fault_t;

typedef struct {
    heater_mode_t mode;
    heater_algorithm_t algorithm;
    heater_fault_t fault;
    bool sensor_ok;
    bool preheat_ready;
    bool ssr_on;
    float temperature_c;
    float target_c;
    float output_percent;
    float slope_c_per_s;
    float kp;
    float ki;
    float kd;
    float calibration_max_slope;
    float calibration_overshoot;
    uint32_t mode_elapsed_ms;
    uint32_t boost_remaining_s;
    uint32_t uptime_s;
} heater_snapshot_t;

esp_err_t heater_control_init(void);
void heater_get_snapshot(heater_snapshot_t *snapshot);

esp_err_t heater_start_constant(float target_c);
esp_err_t heater_start_preheat(float target_c);
esp_err_t heater_start_boost(float target_c, uint32_t duration_s);
esp_err_t heater_start_calibration(void);
esp_err_t heater_stop(void);
esp_err_t heater_emergency_stop(void);
esp_err_t heater_reset_fault(void);
esp_err_t heater_set_algorithm(heater_algorithm_t algorithm);
esp_err_t heater_set_pid(float kp, float ki, float kd);

const char *heater_mode_name(heater_mode_t mode);
const char *heater_fault_name(heater_fault_t fault);
const char *heater_algorithm_name(heater_algorithm_t algorithm);
