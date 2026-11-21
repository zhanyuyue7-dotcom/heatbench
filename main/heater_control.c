#include "heater_control.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "max6675.h"
#include "sdkconfig.h"

#define SAMPLE_PERIOD_MS 250
#define SSR_WINDOW_MS 1000
#define SSR_TASK_PERIOD_MS 10
#define CALIBRATION_HEAT_MS 10000
#define CALIBRATION_MAX_COAST_MS 60000

static const char *TAG = "heater";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static heater_snapshot_t s_state = {
    .mode = HEATER_MODE_IDLE,
    .algorithm = HEATER_ALGORITHM_STAGED,
    .fault = HEATER_FAULT_NONE,
    .sensor_ok = false,
    .temperature_c = NAN,
    .target_c = 0.0f,
    .kp = 2.0f,
    .ki = 0.08f,
    .kd = 6.0f,
};

static led_strip_handle_t s_led_strip;
static atomic_bool s_force_off = ATOMIC_VAR_INIT(true);
static int64_t s_boot_ms;
static int64_t s_mode_started_ms;
static int64_t s_boost_end_ms;
static float s_integral;
static float s_previous_temperature = NAN;
static float s_calibration_temperature_at_cutoff;
static float s_calibration_peak_temperature;
static unsigned s_preheat_stable_samples;
static unsigned s_calibration_falling_samples;

static inline int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static float clampf(float value, float minimum, float maximum)
{
    return fminf(maximum, fmaxf(minimum, value));
}

static void ssr_write(bool enabled)
{
    const int active = CONFIG_HEATER_SSR_ACTIVE_HIGH ? 1 : 0;
    gpio_set_level(CONFIG_HEATER_SSR_GPIO, enabled ? active : !active);
}

static esp_err_t configure_active_output(int gpio, bool active_high)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = active_high ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        .pull_down_en = active_high ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

static bool mode_can_heat(heater_mode_t mode)
{
    return mode == HEATER_MODE_CONSTANT || mode == HEATER_MODE_PREHEAT ||
           mode == HEATER_MODE_BOOST || mode == HEATER_MODE_CALIBRATE_HEAT;
}

static void set_mode_locked(heater_mode_t mode, int64_t timestamp_ms)
{
    s_state.mode = mode;
    s_mode_started_ms = timestamp_ms;
    s_state.mode_elapsed_ms = 0;
    s_state.preheat_ready = false;
    s_preheat_stable_samples = 0;
    s_integral = 0.0f;
}

static void latch_fault_locked(heater_fault_t fault, int64_t timestamp_ms)
{
    if (s_state.fault != HEATER_FAULT_NONE) {
        return;
    }

    s_state.fault = fault;
    s_state.output_percent = 0.0f;
    atomic_store(&s_force_off, true);
    ssr_write(false);
    s_state.ssr_on = false;
    set_mode_locked(HEATER_MODE_FAULT, timestamp_ms);
}

static float staged_output(float temperature_c, float target_c, float slope_c_per_s)
{
    const float predicted_temperature = temperature_c + fmaxf(0.0f, slope_c_per_s) * 2.0f;
    const float error = target_c - predicted_temperature;

    if (error > 20.0f) {
        return 100.0f;
    }
    if (error > 8.0f) {
        return 60.0f;
    }
    if (error > 2.0f) {
        return 25.0f;
    }
    if (error > 0.5f) {
        return 10.0f;
    }
    return 0.0f;
}

static float pid_output_locked(float temperature_c, float target_c, float dt_seconds)
{
    const float error = target_c - temperature_c;
    if (fabsf(error) <= 20.0f) {
        s_integral += error * dt_seconds;
        s_integral = clampf(s_integral, -250.0f, 250.0f);
    } else {
        s_integral *= 0.95f;
    }

    const float derivative_on_measurement = -s_state.slope_c_per_s;
    const float output = s_state.kp * error + s_state.ki * s_integral +
                         s_state.kd * derivative_on_measurement;
    return clampf(output, 0.0f, 100.0f);
}

static float calculate_output_locked(float temperature_c, float dt_seconds)
{
    if (!mode_can_heat(s_state.mode) || s_state.fault != HEATER_FAULT_NONE) {
        return 0.0f;
    }
    if (s_state.mode == HEATER_MODE_CALIBRATE_HEAT) {
        return 100.0f;
    }
    if (s_state.algorithm == HEATER_ALGORITHM_PID) {
        return pid_output_locked(temperature_c, s_state.target_c, dt_seconds);
    }
    return staged_output(temperature_c, s_state.target_c, s_state.slope_c_per_s);
}

static void update_mode_locked(int64_t timestamp_ms, float temperature_c)
{
    const int64_t elapsed_ms = timestamp_ms - s_mode_started_ms;
    s_state.mode_elapsed_ms = elapsed_ms > 0 ? (uint32_t)elapsed_ms : 0;
    s_state.boost_remaining_s = 0;

    switch (s_state.mode) {
    case HEATER_MODE_PREHEAT:
        if (fabsf(s_state.target_c - temperature_c) <= 1.5f &&
            fabsf(s_state.slope_c_per_s) <= 1.0f) {
            s_preheat_stable_samples++;
            if (!s_state.preheat_ready && s_preheat_stable_samples >= 8) {
                s_state.preheat_ready = true;
                ESP_LOGI(TAG, "Preheat ready at %.1f C; waiting for Boost command", temperature_c);
            }
        } else {
            s_preheat_stable_samples = 0;
            s_state.preheat_ready = false;
        }
        break;

    case HEATER_MODE_BOOST:
        if (timestamp_ms >= s_boost_end_ms) {
            set_mode_locked(HEATER_MODE_COOLDOWN, timestamp_ms);
            s_state.target_c = 0.0f;
            ESP_LOGI(TAG, "Boost timer complete; cooling started");
        } else {
            s_state.boost_remaining_s = (uint32_t)((s_boost_end_ms - timestamp_ms + 999) / 1000);
        }
        break;

    case HEATER_MODE_COOLDOWN:
        if (temperature_c <= CONFIG_HEATER_RESIDUAL_WARNING_C) {
            set_mode_locked(HEATER_MODE_IDLE, timestamp_ms);
            ESP_LOGI(TAG, "Plate cooled below residual-heat threshold");
        }
        break;

    case HEATER_MODE_CALIBRATE_HEAT:
        s_state.calibration_max_slope = fmaxf(s_state.calibration_max_slope,
                                               s_state.slope_c_per_s);
        s_calibration_peak_temperature = fmaxf(s_calibration_peak_temperature,
                                                temperature_c);
        if (elapsed_ms >= CALIBRATION_HEAT_MS) {
            s_calibration_temperature_at_cutoff = temperature_c;
            s_calibration_peak_temperature = temperature_c;
            s_calibration_falling_samples = 0;
            set_mode_locked(HEATER_MODE_CALIBRATE_COAST, timestamp_ms);
            ESP_LOGI(TAG, "Calibration heat pulse complete at %.1f C; measuring coast",
                     temperature_c);
        }
        break;

    case HEATER_MODE_CALIBRATE_COAST:
        s_calibration_peak_temperature = fmaxf(s_calibration_peak_temperature,
                                                temperature_c);
        s_state.calibration_overshoot = fmaxf(0.0f, s_calibration_peak_temperature -
                                                       s_calibration_temperature_at_cutoff);
        if (s_state.slope_c_per_s < -0.05f) {
            s_calibration_falling_samples++;
        } else {
            s_calibration_falling_samples = 0;
        }
        if ((elapsed_ms >= 5000 && s_calibration_falling_samples >= 8) ||
            elapsed_ms >= CALIBRATION_MAX_COAST_MS) {
            set_mode_locked(temperature_c > CONFIG_HEATER_RESIDUAL_WARNING_C
                                ? HEATER_MODE_COOLDOWN
                                : HEATER_MODE_IDLE,
                            timestamp_ms);
            ESP_LOGI(TAG, "Calibration complete: max slope %.2f C/s, overshoot %.2f C",
                     s_state.calibration_max_slope, s_state.calibration_overshoot);
        }
        break;

    default:
        break;
    }
}

static void update_indicator(int64_t timestamp_ms, const heater_snapshot_t *snapshot)
{
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;

    if (snapshot->mode == HEATER_MODE_FAULT) {
        red = ((timestamp_ms / 250) % 2) ? 32 : 2;
    } else if (mode_can_heat(snapshot->mode) && snapshot->output_percent > 0.0f) {
        red = 28;
        green = 1;
    } else if (snapshot->temperature_c > CONFIG_HEATER_RESIDUAL_WARNING_C) {
        const uint8_t phase = (uint8_t)((timestamp_ms / 60) % 32);
        const uint8_t level = phase < 16 ? (uint8_t)(5 + phase) : (uint8_t)(36 - phase);
        red = level;
        green = (uint8_t)(level / 5);
    } else {
        blue = 18;
        green = 3;
    }

    if (s_led_strip != NULL) {
        for (int pixel = 0; pixel < CONFIG_HEATER_RGB_LED_COUNT; ++pixel) {
            led_strip_set_pixel(s_led_strip, pixel, red, green, blue);
        }
        led_strip_refresh(s_led_strip);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    esp_task_wdt_add(NULL);

    // The MAX6675 needs one complete conversion before the first meaningful read.
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        const int64_t timestamp_ms = now_ms();
        max6675_sample_t sample = {0};
        const esp_err_t read_result = max6675_read(&sample);

        portENTER_CRITICAL(&s_lock);
        const heater_fault_t previous_fault = s_state.fault;
        s_state.uptime_s = (uint32_t)((timestamp_ms - s_boot_ms) / 1000);

        if (read_result != ESP_OK) {
            s_state.sensor_ok = false;
            latch_fault_locked(HEATER_FAULT_INTERNAL, timestamp_ms);
        } else if (sample.thermocouple_open) {
            s_state.sensor_ok = false;
            latch_fault_locked(HEATER_FAULT_THERMOCOUPLE_OPEN, timestamp_ms);
        } else if (sample.temperature_c >= CONFIG_HEATER_MAX_TEMPERATURE_C) {
            s_state.sensor_ok = true;
            s_state.temperature_c = sample.temperature_c;
            latch_fault_locked(HEATER_FAULT_OVERTEMPERATURE, timestamp_ms);
        } else {
            s_state.sensor_ok = true;
            s_state.temperature_c = sample.temperature_c;

            if (!isnan(s_previous_temperature)) {
                const float raw_slope = (sample.temperature_c - s_previous_temperature) /
                                        (SAMPLE_PERIOD_MS / 1000.0f);
                s_state.slope_c_per_s = 0.65f * s_state.slope_c_per_s + 0.35f * raw_slope;
            }
            s_previous_temperature = sample.temperature_c;

            if (s_state.fault == HEATER_FAULT_NONE) {
                update_mode_locked(timestamp_ms, sample.temperature_c);
                s_state.output_percent = calculate_output_locked(
                    sample.temperature_c, SAMPLE_PERIOD_MS / 1000.0f);
            }
        }

        if (s_state.fault != HEATER_FAULT_NONE) {
            s_state.output_percent = 0.0f;
        }

        heater_snapshot_t snapshot = s_state;
        portEXIT_CRITICAL(&s_lock);

        if (previous_fault == HEATER_FAULT_NONE && snapshot.fault != HEATER_FAULT_NONE) {
            ESP_LOGE(TAG, "Safety lockout: %s", heater_fault_name(snapshot.fault));
        }
        update_indicator(timestamp_ms, &snapshot);

        esp_task_wdt_reset();
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

static void ssr_task(void *arg)
{
    (void)arg;
    int64_t window_start_ms = now_ms();
    bool previous_level = false;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        const int64_t timestamp_ms = now_ms();
        if (timestamp_ms - window_start_ms >= SSR_WINDOW_MS) {
            window_start_ms += ((timestamp_ms - window_start_ms) / SSR_WINDOW_MS) * SSR_WINDOW_MS;
        }

        heater_snapshot_t snapshot;
        heater_get_snapshot(&snapshot);

        const uint32_t on_time_ms = (uint32_t)lroundf(
            clampf(snapshot.output_percent, 0.0f, 100.0f) * (SSR_WINDOW_MS / 100.0f));
        const bool requested_level = (uint32_t)(timestamp_ms - window_start_ms) < on_time_ms;
        portENTER_CRITICAL(&s_lock);
        const bool safe_to_heat = !atomic_load(&s_force_off) && s_state.sensor_ok &&
                                  s_state.fault == HEATER_FAULT_NONE && mode_can_heat(s_state.mode);
        const bool level = safe_to_heat && requested_level;
        if (level != previous_level) {
            ssr_write(level);
            previous_level = level;
        }
        s_state.ssr_on = level;
        portEXIT_CRITICAL(&s_lock);

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SSR_TASK_PERIOD_MS));
    }
}

esp_err_t heater_control_init(void)
{
    s_boot_ms = now_ms();
    s_mode_started_ms = s_boot_ms;

    const int pins[] = {
        CONFIG_HEATER_MAX6675_SCK_GPIO,
        CONFIG_HEATER_MAX6675_CS_GPIO,
        CONFIG_HEATER_MAX6675_SO_GPIO,
        CONFIG_HEATER_SSR_GPIO,
        CONFIG_HEATER_RGB_GPIO,
    };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(pins[i]), ESP_ERR_INVALID_ARG, TAG,
                            "GPIO %d is not valid for ESP32-S3", pins[i]);
        for (size_t j = i + 1; j < sizeof(pins) / sizeof(pins[0]); ++j) {
            ESP_RETURN_ON_FALSE(pins[i] != pins[j], ESP_ERR_INVALID_ARG, TAG,
                                "GPIO %d is assigned to more than one function", pins[i]);
        }
    }
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_HEATER_MAX6675_SCK_GPIO) &&
                            GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_HEATER_MAX6675_CS_GPIO) &&
                            GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_HEATER_SSR_GPIO) &&
                            GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_HEATER_RGB_GPIO),
                        ESP_ERR_INVALID_ARG, TAG, "one or more configured outputs use input-only GPIOs");

    ESP_RETURN_ON_ERROR(configure_active_output(CONFIG_HEATER_SSR_GPIO,
                                                 CONFIG_HEATER_SSR_ACTIVE_HIGH),
                        TAG, "SSR GPIO setup failed");
    ssr_write(false);

    ESP_RETURN_ON_ERROR(max6675_init(), TAG, "MAX6675 initialization failed");

    const led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_HEATER_RGB_GPIO,
        .max_leds = CONFIG_HEATER_RGB_LED_COUNT,
    };
    const led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    esp_err_t led_result = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (led_result != ESP_OK) {
        ESP_LOGW(TAG, "RGB status LED unavailable: %s", esp_err_to_name(led_result));
        s_led_strip = NULL;
    } else {
        led_strip_clear(s_led_strip);
    }

    BaseType_t task_result = xTaskCreatePinnedToCore(control_task, "heater_control", 5120, NULL,
                                                      12, NULL, 1);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "control task creation failed");
    task_result = xTaskCreatePinnedToCore(ssr_task, "ssr_window", 3072, NULL, 13, NULL, 1);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "SSR task creation failed");

    ESP_LOGI(TAG, "Controller ready: SSR=%d RGB=%d x%d; SSR window=%d ms",
             CONFIG_HEATER_SSR_GPIO, CONFIG_HEATER_RGB_GPIO,
             CONFIG_HEATER_RGB_LED_COUNT, SSR_WINDOW_MS);
    return ESP_OK;
}

void heater_get_snapshot(heater_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *snapshot = s_state;
    portEXIT_CRITICAL(&s_lock);
}

static esp_err_t validate_start_locked(float target_c)
{
    if (!s_state.sensor_ok || s_state.fault != HEATER_FAULT_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(target_c) || target_c < 40.0f || target_c > 250.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t heater_start_constant(float target_c)
{
    esp_err_t result;
    const int64_t timestamp_ms = now_ms();
    portENTER_CRITICAL(&s_lock);
    result = validate_start_locked(target_c);
    if (result == ESP_OK) {
        s_state.target_c = target_c;
        set_mode_locked(HEATER_MODE_CONSTANT, timestamp_ms);
    }
    portEXIT_CRITICAL(&s_lock);
    if (result == ESP_OK) {
        atomic_store(&s_force_off, false);
    }
    return result;
}

esp_err_t heater_start_preheat(float target_c)
{
    esp_err_t result;
    const int64_t timestamp_ms = now_ms();
    portENTER_CRITICAL(&s_lock);
    result = validate_start_locked(target_c);
    if (result == ESP_OK && (target_c < 140.0f || target_c > 150.0f)) {
        result = ESP_ERR_INVALID_ARG;
    }
    if (result == ESP_OK) {
        s_state.target_c = target_c;
        set_mode_locked(HEATER_MODE_PREHEAT, timestamp_ms);
    }
    portEXIT_CRITICAL(&s_lock);
    if (result == ESP_OK) {
        atomic_store(&s_force_off, false);
    }
    return result;
}

esp_err_t heater_start_boost(float target_c, uint32_t duration_s)
{
    esp_err_t result = ESP_OK;
    const int64_t timestamp_ms = now_ms();
    portENTER_CRITICAL(&s_lock);
    if (!s_state.sensor_ok || s_state.fault != HEATER_FAULT_NONE ||
        s_state.mode != HEATER_MODE_PREHEAT || !s_state.preheat_ready) {
        result = ESP_ERR_INVALID_STATE;
    } else if (!isfinite(target_c) || target_c < 220.0f || target_c > 250.0f ||
               duration_s < 5 || duration_s > 180) {
        result = ESP_ERR_INVALID_ARG;
    } else {
        s_state.target_c = target_c;
        s_boost_end_ms = timestamp_ms + (int64_t)duration_s * 1000;
        set_mode_locked(HEATER_MODE_BOOST, timestamp_ms);
    }
    portEXIT_CRITICAL(&s_lock);
    if (result == ESP_OK) {
        atomic_store(&s_force_off, false);
    }
    return result;
}

esp_err_t heater_start_calibration(void)
{
    esp_err_t result = ESP_OK;
    const int64_t timestamp_ms = now_ms();
    portENTER_CRITICAL(&s_lock);
    if (!s_state.sensor_ok || s_state.fault != HEATER_FAULT_NONE ||
        s_state.temperature_c > 80.0f) {
        result = ESP_ERR_INVALID_STATE;
    } else {
        s_state.target_c = 0.0f;
        s_state.calibration_max_slope = 0.0f;
        s_state.calibration_overshoot = 0.0f;
        s_calibration_temperature_at_cutoff = s_state.temperature_c;
        s_calibration_peak_temperature = s_state.temperature_c;
        set_mode_locked(HEATER_MODE_CALIBRATE_HEAT, timestamp_ms);
    }
    portEXIT_CRITICAL(&s_lock);
    if (result == ESP_OK) {
        atomic_store(&s_force_off, false);
    }
    return result;
}

esp_err_t heater_stop(void)
{
    const int64_t timestamp_ms = now_ms();
    atomic_store(&s_force_off, true);
    portENTER_CRITICAL(&s_lock);
    ssr_write(false);
    s_state.ssr_on = false;
    s_state.output_percent = 0.0f;
    if (s_state.fault == HEATER_FAULT_NONE) {
        set_mode_locked(s_state.sensor_ok &&
                                s_state.temperature_c > CONFIG_HEATER_RESIDUAL_WARNING_C
                            ? HEATER_MODE_COOLDOWN
                            : HEATER_MODE_IDLE,
                        timestamp_ms);
        s_state.target_c = 0.0f;
    }
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t heater_emergency_stop(void)
{
    const int64_t timestamp_ms = now_ms();
    atomic_store(&s_force_off, true);
    portENTER_CRITICAL(&s_lock);
    ssr_write(false);
    s_state.ssr_on = false;
    s_state.output_percent = 0.0f;
    latch_fault_locked(HEATER_FAULT_USER_ESTOP, timestamp_ms);
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGE(TAG, "Safety lockout: %s", heater_fault_name(HEATER_FAULT_USER_ESTOP));
    return ESP_OK;
}

esp_err_t heater_reset_fault(void)
{
    esp_err_t result = ESP_OK;
    const int64_t timestamp_ms = now_ms();
    portENTER_CRITICAL(&s_lock);
    if (!s_state.sensor_ok || !isfinite(s_state.temperature_c) ||
        s_state.temperature_c >= CONFIG_HEATER_MAX_TEMPERATURE_C - 10) {
        result = ESP_ERR_INVALID_STATE;
    } else {
        s_state.fault = HEATER_FAULT_NONE;
        s_state.output_percent = 0.0f;
        atomic_store(&s_force_off, true);
        set_mode_locked(s_state.temperature_c > CONFIG_HEATER_RESIDUAL_WARNING_C
                            ? HEATER_MODE_COOLDOWN
                            : HEATER_MODE_IDLE,
                        timestamp_ms);
        s_state.target_c = 0.0f;
        s_previous_temperature = s_state.temperature_c;
    }
    portEXIT_CRITICAL(&s_lock);
    return result;
}

esp_err_t heater_set_algorithm(heater_algorithm_t algorithm)
{
    if (algorithm != HEATER_ALGORITHM_STAGED && algorithm != HEATER_ALGORITHM_PID) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_lock);
    s_state.algorithm = algorithm;
    s_integral = 0.0f;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t heater_set_pid(float kp, float ki, float kd)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        kp < 0.0f || kp > 20.0f || ki < 0.0f || ki > 5.0f || kd < 0.0f || kd > 50.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_lock);
    s_state.kp = kp;
    s_state.ki = ki;
    s_state.kd = kd;
    s_integral = 0.0f;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

const char *heater_mode_name(heater_mode_t mode)
{
    switch (mode) {
    case HEATER_MODE_IDLE: return "idle";
    case HEATER_MODE_CONSTANT: return "constant";
    case HEATER_MODE_PREHEAT: return "preheat";
    case HEATER_MODE_BOOST: return "boost";
    case HEATER_MODE_COOLDOWN: return "cooldown";
    case HEATER_MODE_CALIBRATE_HEAT: return "calibrate_heat";
    case HEATER_MODE_CALIBRATE_COAST: return "calibrate_coast";
    case HEATER_MODE_FAULT: return "fault";
    default: return "unknown";
    }
}

const char *heater_fault_name(heater_fault_t fault)
{
    switch (fault) {
    case HEATER_FAULT_NONE: return "none";
    case HEATER_FAULT_THERMOCOUPLE_OPEN: return "thermocouple_open";
    case HEATER_FAULT_OVERTEMPERATURE: return "overtemperature";
    case HEATER_FAULT_USER_ESTOP: return "user_estop";
    case HEATER_FAULT_INTERNAL: return "internal_error";
    default: return "unknown";
    }
}

const char *heater_algorithm_name(heater_algorithm_t algorithm)
{
    return algorithm == HEATER_ALGORITHM_PID ? "pid" : "staged";
}
