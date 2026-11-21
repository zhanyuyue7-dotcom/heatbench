#include "web_server.h"

#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heater_control.h"

static const char *TAG = "web";
static httpd_handle_t s_server;

extern const unsigned char web_index_html_start[] asm("_binary_index_html_start");
extern const unsigned char web_index_html_end[] asm("_binary_index_html_end");

static char *create_telemetry_json(void)
{
    heater_snapshot_t snapshot;
    heater_get_snapshot(&snapshot);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return NULL;
    }

    cJSON_AddStringToObject(root, "type", "telemetry");
    cJSON_AddStringToObject(root, "mode", heater_mode_name(snapshot.mode));
    cJSON_AddStringToObject(root, "algorithm", heater_algorithm_name(snapshot.algorithm));
    cJSON_AddStringToObject(root, "fault", heater_fault_name(snapshot.fault));
    cJSON_AddBoolToObject(root, "sensorOk", snapshot.sensor_ok);
    cJSON_AddBoolToObject(root, "preheatReady", snapshot.preheat_ready);
    cJSON_AddBoolToObject(root, "ssrOn", snapshot.ssr_on);
    if (snapshot.sensor_ok) {
        cJSON_AddNumberToObject(root, "temperature", snapshot.temperature_c);
    } else {
        cJSON_AddNullToObject(root, "temperature");
    }
    cJSON_AddNumberToObject(root, "target", snapshot.target_c);
    cJSON_AddNumberToObject(root, "output", snapshot.output_percent);
    cJSON_AddNumberToObject(root, "slope", snapshot.slope_c_per_s);
    cJSON_AddNumberToObject(root, "kp", snapshot.kp);
    cJSON_AddNumberToObject(root, "ki", snapshot.ki);
    cJSON_AddNumberToObject(root, "kd", snapshot.kd);
    cJSON_AddNumberToObject(root, "calibrationMaxSlope", snapshot.calibration_max_slope);
    cJSON_AddNumberToObject(root, "calibrationOvershoot", snapshot.calibration_overshoot);
    cJSON_AddNumberToObject(root, "modeElapsedMs", snapshot.mode_elapsed_ms);
    cJSON_AddNumberToObject(root, "boostRemainingS", snapshot.boost_remaining_s);
    cJSON_AddNumberToObject(root, "uptimeS", snapshot.uptime_s);

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return payload;
}

static esp_err_t root_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    const size_t length = web_index_html_end - web_index_html_start;
    return httpd_resp_send(request, (const char *)web_index_html_start, length);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    char *payload = create_telemetry_json();
    if (payload == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "telemetry allocation failed");
    }
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    const esp_err_t result = httpd_resp_sendstr(request, payload);
    free(payload);
    return result;
}

static esp_err_t send_ack(httpd_req_t *request, bool ok, const char *command,
                          const char *message)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddStringToObject(root, "type", "ack");
    cJSON_AddBoolToObject(root, "ok", ok);
    cJSON_AddStringToObject(root, "command", command != NULL ? command : "unknown");
    cJSON_AddStringToObject(root, "message", message != NULL ? message : "");
    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (payload == NULL) {
        return ESP_ERR_NO_MEM;
    }

    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)payload,
        .len = strlen(payload),
    };
    const esp_err_t result = httpd_ws_send_frame(request, &frame);
    free(payload);
    return result;
}

static bool json_number(const cJSON *root, const char *name, double *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    if (!cJSON_IsNumber(item)) {
        return false;
    }
    *value = item->valuedouble;
    return true;
}

static esp_err_t execute_command(const cJSON *root, const char *command)
{
    double target = 0.0;
    double duration = 0.0;

    if (strcmp(command, "constant") == 0) {
        return json_number(root, "target", &target)
                   ? heater_start_constant((float)target)
                   : ESP_ERR_INVALID_ARG;
    }
    if (strcmp(command, "preheat") == 0) {
        return json_number(root, "target", &target)
                   ? heater_start_preheat((float)target)
                   : ESP_ERR_INVALID_ARG;
    }
    if (strcmp(command, "boost") == 0) {
        return json_number(root, "target", &target) && json_number(root, "duration", &duration)
                   ? heater_start_boost((float)target, (uint32_t)duration)
                   : ESP_ERR_INVALID_ARG;
    }
    if (strcmp(command, "calibrate") == 0) {
        return heater_start_calibration();
    }
    if (strcmp(command, "stop") == 0) {
        return heater_stop();
    }
    if (strcmp(command, "estop") == 0) {
        return heater_emergency_stop();
    }
    if (strcmp(command, "reset_fault") == 0) {
        return heater_reset_fault();
    }
    if (strcmp(command, "algorithm") == 0) {
        const cJSON *value = cJSON_GetObjectItemCaseSensitive(root, "value");
        if (!cJSON_IsString(value)) {
            return ESP_ERR_INVALID_ARG;
        }
        if (strcmp(value->valuestring, "pid") == 0) {
            return heater_set_algorithm(HEATER_ALGORITHM_PID);
        }
        if (strcmp(value->valuestring, "staged") == 0) {
            return heater_set_algorithm(HEATER_ALGORITHM_STAGED);
        }
        return ESP_ERR_INVALID_ARG;
    }
    if (strcmp(command, "pid") == 0) {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        return json_number(root, "kp", &kp) && json_number(root, "ki", &ki) &&
                       json_number(root, "kd", &kd)
                   ? heater_set_pid((float)kp, (float)ki, (float)kd)
                   : ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t websocket_handler(httpd_req_t *request)
{
    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t result = httpd_ws_recv_frame(request, &frame, 0);
    if (result != ESP_OK) {
        return result;
    }
    if (frame.len == 0 || frame.len > 512) {
        return send_ack(request, false, "unknown", "Command payload is empty or too large");
    }

    uint8_t *buffer = calloc(1, frame.len + 1);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = buffer;
    result = httpd_ws_recv_frame(request, &frame, frame.len);
    if (result != ESP_OK) {
        free(buffer);
        return result;
    }
    if (frame.type != HTTPD_WS_TYPE_TEXT) {
        free(buffer);
        return send_ack(request, false, "unknown", "Only text commands are accepted");
    }

    cJSON *root = cJSON_Parse((char *)buffer);
    free(buffer);
    if (root == NULL) {
        return send_ack(request, false, "unknown", "Invalid JSON");
    }

    const cJSON *command_item = cJSON_GetObjectItemCaseSensitive(root, "command");
    if (!cJSON_IsString(command_item)) {
        cJSON_Delete(root);
        return send_ack(request, false, "unknown", "Missing command name");
    }

    char command[24];
    strlcpy(command, command_item->valuestring, sizeof(command));
    result = execute_command(root, command);
    cJSON_Delete(root);

    const char *message = "Command accepted";
    if (result == ESP_ERR_INVALID_ARG) {
        message = "Parameter is outside the allowed range";
    } else if (result == ESP_ERR_INVALID_STATE) {
        message = "Current safety or process state does not allow this command";
    } else if (result == ESP_ERR_NOT_SUPPORTED) {
        message = "Unknown command";
    } else if (result != ESP_OK) {
        message = "Command failed internally";
    }
    return send_ack(request, result == ESP_OK, command, message);
}

typedef struct {
    char *payload;
} broadcast_job_t;

static void broadcast_work(void *context)
{
    broadcast_job_t *job = context;
    int client_fds[8];
    size_t client_count = sizeof(client_fds) / sizeof(client_fds[0]);
    if (httpd_get_client_list(s_server, &client_count, client_fds) == ESP_OK) {
        httpd_ws_frame_t frame = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)job->payload,
            .len = strlen(job->payload),
        };
        for (size_t i = 0; i < client_count; ++i) {
            if (httpd_ws_get_fd_info(s_server, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                httpd_ws_send_frame_async(s_server, client_fds[i], &frame);
            }
        }
    }
    free(job->payload);
    free(job);
}

static void telemetry_task(void *arg)
{
    (void)arg;
    while (true) {
        broadcast_job_t *job = calloc(1, sizeof(*job));
        if (job != NULL) {
            job->payload = create_telemetry_json();
            if (job->payload == NULL || httpd_queue_work(s_server, broadcast_work, job) != ESP_OK) {
                free(job->payload);
                free(job);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

esp_err_t web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 6;
    config.lru_purge_enable = true;
    config.stack_size = 6144;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG, "HTTP server start failed");

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
    };
    const httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_handler,
    };
    const httpd_uri_t websocket_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .is_websocket = true,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &root_uri), TAG,
                        "root route registration failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &status_uri), TAG,
                        "status route registration failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &websocket_uri), TAG,
                        "WebSocket route registration failed");

    BaseType_t task_result = xTaskCreatePinnedToCore(telemetry_task, "web_telemetry", 4096,
                                                      NULL, 5, NULL, 0);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "telemetry task creation failed");

    ESP_LOGI(TAG, "Dashboard server ready");
    return ESP_OK;
}
