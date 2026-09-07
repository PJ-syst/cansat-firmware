#include <math.h>
#include <stdio.h>
#include <string.h>

#include "battery.h"
#include "cansat_types.h"
#include "command_parser.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "flight_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gps_uart.h"
#include "mechanism.h"
#include "nmea_parser.h"
#include "radio.h"
#include "sdkconfig.h"
#include "sensors.h"
#include "telemetry_formatter.h"

#define ALTITUDE_QUEUE_DEPTH 32U
#define COMMAND_QUEUE_DEPTH 8U
#define MECHANISM_AUTO_BIT (1UL << 0)
#define MECHANISM_BENCH_BIT (1UL << 1)
#define GPS_STALE_MS 5000U
#define SENSOR_IMPLAUSIBLE_STEP_M 500.0f
#define SENSOR_RECOVERY_FAILURES 20U
#define SENSOR_RECOVERY_MAX_FAILURES 600U
#define UART_RECOVERY_DELAY_MS 5000U
#define DIAGNOSTIC_PERIOD_MS 60000U

typedef struct {
    char data[CANSAT_COMMAND_MAX_LEN];
    size_t length;
} raw_command_frame_t;

typedef struct {
    bool telemetry_enabled;
    bool simulation_enabled;
    bool simulation_active;
    bool simulation_pressure_valid;
    float simulation_pressure_kpa;
    uint32_t cxon_epoch_ms;
    char command_echo[CANSAT_CMD_ECHO_MAX_LEN];
} control_state_t;

typedef struct {
    bool gps_synced;
    uint32_t gps_seconds_of_day;
    uint32_t gps_sync_ms;
} mission_clock_t;

static const char *TAG = "cansat";
static QueueHandle_t s_sensor_latest_queue;
static QueueHandle_t s_altitude_queue;
static QueueHandle_t s_command_queue;
static QueueHandle_t s_mechanism_result_queue;
static SemaphoreHandle_t s_gps_mutex;
static SemaphoreHandle_t s_state_mutex;
static SemaphoreHandle_t s_control_mutex;
static SemaphoreHandle_t s_radio_mutex;
static TaskHandle_t s_mechanism_task_handle;
static gps_data_t s_gps;
static flight_state_machine_t s_flight;
static control_state_t s_control;

static uint32_t uptime_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static void copy_control(control_state_t *destination)
{
    xSemaphoreTake(s_control_mutex, portMAX_DELAY);
    *destination = s_control;
    xSemaphoreGive(s_control_mutex);
}

static void sensor_sample_task(void *argument)
{
    (void)argument;
    (void)esp_task_wdt_add(NULL);
    TickType_t last_wake = xTaskGetTickCount();
    float last_valid_altitude_m = 0.0f;
    bool last_valid_altitude_available = false;
    uint32_t rejected_samples = 0U;
    uint32_t hardware_failures = 0U;
    uint32_t recovery_threshold = SENSOR_RECOVERY_FAILURES;
    uint32_t recovery_attempts = 0U;
    for (;;) {
        sensor_reading_t reading;
        const esp_err_t result = sensors_read(&reading);
        control_state_t control;
        copy_control(&control);

        if (control.simulation_active && control.simulation_pressure_valid) {
            reading.pressure_kpa = control.simulation_pressure_kpa;
            reading.altitude_m = sensors_altitude_from_pressure_kpa(reading.pressure_kpa);
            reading.pressure_valid = true;
            reading.altitude_valid = isfinite(reading.altitude_m);
        }

        if (result != ESP_OK && !reading.altitude_valid &&
            (++rejected_samples == 1U || rejected_samples % 50U == 0U)) {
            ESP_LOGW(TAG, "barometer sample rejected: %s", esp_err_to_name(result));
        }
        if (result != ESP_OK) {
            ++hardware_failures;
            if (hardware_failures >= recovery_threshold) {
                sensors_health_t health;
                const esp_err_t recovery = sensors_recover(&health);
                ++recovery_attempts;
                if (recovery_attempts == 1U || recovery_attempts % 10U == 0U) {
                    ESP_LOGW(TAG, "sensor bus recovery attempt %lu: %s",
                             (unsigned long)recovery_attempts, esp_err_to_name(recovery));
                }
                if (recovery == ESP_OK) {
                    recovery_threshold = SENSOR_RECOVERY_FAILURES;
                } else if (recovery_threshold < SENSOR_RECOVERY_MAX_FAILURES / 2U) {
                    recovery_threshold *= 2U;
                } else {
                    recovery_threshold = SENSOR_RECOVERY_MAX_FAILURES;
                }
                hardware_failures = 0U;
            }
        } else {
            hardware_failures = 0U;
            recovery_threshold = SENSOR_RECOVERY_FAILURES;
            recovery_attempts = 0U;
        }
        if (reading.altitude_valid && last_valid_altitude_available &&
            fabsf(reading.altitude_m - last_valid_altitude_m) > SENSOR_IMPLAUSIBLE_STEP_M) {
            ESP_LOGW(TAG, "implausible altitude step rejected");
            reading.altitude_valid = false;
        }
        if (reading.altitude_valid) {
            last_valid_altitude_m = reading.altitude_m;
            last_valid_altitude_available = true;
            rejected_samples = 0U;
        }

        xQueueOverwrite(s_sensor_latest_queue, &reading);
        static uint32_t queue_overruns;
        if (xQueueSend(s_altitude_queue, &reading, 0) != pdPASS) {
            if (++queue_overruns == 1U || queue_overruns % 100U == 0U) {
                ESP_LOGW(TAG, "altitude queue overruns: %lu",
                         (unsigned long)queue_overruns);
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_CANSAT_SENSOR_PERIOD_MS));
        (void)esp_task_wdt_reset();
    }
}

static void flight_state_task(void *argument)
{
    (void)argument;
    (void)esp_task_wdt_add(NULL);
    for (;;) {
        (void)esp_task_wdt_reset();
        bool mechanism_success;
        if (xQueueReceive(s_mechanism_result_queue, &mechanism_success, 0) == pdPASS) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            const flight_event_t event = flight_state_report_mechanism(&s_flight,
                                                                        mechanism_success);
            const flight_state_t state = s_flight.state;
            xSemaphoreGive(s_state_mutex);
            if ((event & FLIGHT_EVENT_STATE_CHANGED) != 0U) {
                ESP_LOGI(TAG, "flight state: %s", cansat_state_name(state));
            }
        }

        sensor_reading_t reading;
        if (xQueueReceive(s_altitude_queue, &reading, pdMS_TO_TICKS(200)) != pdPASS) {
            continue;
        }
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        const flight_state_t before = s_flight.state;
        const flight_event_t event = flight_state_process(&s_flight, reading.altitude_m,
                                                           reading.altitude_valid);
        const flight_state_t after = s_flight.state;
        xSemaphoreGive(s_state_mutex);
        if ((event & FLIGHT_EVENT_STATE_CHANGED) != 0U && before != after) {
            ESP_LOGI(TAG, "flight state: %s", cansat_state_name(after));
        }
        if ((event & FLIGHT_EVENT_REQUEST_MECHANISM) != 0U) {
            xTaskNotify(s_mechanism_task_handle, MECHANISM_AUTO_BIT, eSetBits);
        }
    }
}

static void gps_task(void *argument)
{
    (void)argument;
    char sentence[128];
    size_t used = 0U;
    bool collecting = false;
    uint8_t bytes[64];
    uint32_t failures = 0U;
    for (;;) {
        const int count = gps_uart_read(bytes, sizeof(bytes), 200U);
        if (count < 0) {
            const esp_err_t recovery = gps_uart_recover();
            if (++failures == 1U || failures % 10U == 0U) {
                ESP_LOGW(TAG, "GPS UART recovery: %s", esp_err_to_name(recovery));
            }
            vTaskDelay(pdMS_TO_TICKS(UART_RECOVERY_DELAY_MS));
            continue;
        }
        if (count == 0) continue;
        failures = 0U;
        for (int i = 0; i < count; ++i) {
            const char c = (char)bytes[i];
            if (c == '$') {
                used = 0U;
                collecting = true;
                sentence[used++] = c;
                continue;
            }
            if (!collecting) continue;
            if (c == '\r' || c == '\n') {
                if (used > 0U) {
                    sentence[used] = '\0';
                    xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
                    (void)nmea_parse_sentence(sentence, uptime_ms(), &s_gps, NULL);
                    xSemaphoreGive(s_gps_mutex);
                }
                collecting = false;
                used = 0U;
            } else if (used + 1U < sizeof(sentence)) {
                sentence[used++] = c;
            } else {
                collecting = false;
                used = 0U;
            }
        }
    }
}

static void radio_rx_task(void *argument)
{
    (void)argument;
    raw_command_frame_t frame = {0};
    bool overflow = false;
    uint8_t bytes[64];
    uint32_t failures = 0U;
    for (;;) {
        xSemaphoreTake(s_radio_mutex, portMAX_DELAY);
        const int count = radio_read(bytes, sizeof(bytes), 200U);
        if (count < 0) {
            const esp_err_t recovery = radio_recover();
            xSemaphoreGive(s_radio_mutex);
            if (++failures == 1U || failures % 10U == 0U) {
                ESP_LOGW(TAG, "radio UART recovery: %s", esp_err_to_name(recovery));
            }
            vTaskDelay(pdMS_TO_TICKS(UART_RECOVERY_DELAY_MS));
            continue;
        }
        xSemaphoreGive(s_radio_mutex);
        if (count == 0) continue;
        failures = 0U;
        for (int i = 0; i < count; ++i) {
            const char c = (char)bytes[i];
            if (c == '\r' || c == '\n') {
                if (!overflow && frame.length > 0U) {
                    frame.data[frame.length] = '\0';
                    if (xQueueSend(s_command_queue, &frame, 0) != pdPASS) {
                        ESP_LOGW(TAG, "command queue full");
                    }
                }
                frame.length = 0U;
                overflow = false;
            } else if (!overflow && frame.length + 1U < sizeof(frame.data)) {
                frame.data[frame.length++] = c;
            } else {
                overflow = true;
            }
        }
    }
}

static bool apply_command(const command_t *command)
{
    bool accepted = true;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    const flight_state_t flight_state = s_flight.state;
    xSemaphoreGive(s_state_mutex);
    const bool prelaunch = flight_state == CANSAT_STATE_STARTUP ||
                           flight_state == CANSAT_STATE_LAUNCH_PAD;
    xSemaphoreTake(s_control_mutex, portMAX_DELAY); 
    switch (command->type) {
    case CANSAT_CMD_CXON:
        s_control.telemetry_enabled = true;
        s_control.cxon_epoch_ms = uptime_ms();
        break;
    case CANSAT_CMD_CXOFF:
        s_control.telemetry_enabled = false;
        break;
    case CANSAT_CMD_SIM_ENABLE:
        if (prelaunch) s_control.simulation_enabled = true;
        else accepted = false;
        break;
    case CANSAT_CMD_SIM_ACTIVATE:
        if (prelaunch && s_control.simulation_enabled) s_control.simulation_active = true;
        else accepted = false;
        break;
    case CANSAT_CMD_SIM_DISABLE:
        if (prelaunch) {
            s_control.simulation_enabled = false;
            s_control.simulation_active = false;
            s_control.simulation_pressure_valid = false;
        } else {
            accepted = false;
        }
        break;
    case CANSAT_CMD_SIM_PRESSURE:
        if (s_control.simulation_enabled) {
            s_control.simulation_pressure_kpa = command->pressure_kpa;
            s_control.simulation_pressure_valid = true;
        } else {
            accepted = false;
        }
        break;
    case CANSAT_CMD_MEC:
#if CONFIG_CANSAT_ALLOW_BENCH_MEC
        break;
#else
        accepted = false;
#endif
        break;
    default:
        accepted = false;
        break;
    }
    if (accepted) {
        (void)snprintf(s_control.command_echo, sizeof(s_control.command_echo),
                       "%s", command->echo);
    }
    xSemaphoreGive(s_control_mutex);

    if (accepted && command->type == CANSAT_CMD_MEC) {
        xTaskNotify(s_mechanism_task_handle, MECHANISM_BENCH_BIT, eSetBits);
    }
    return accepted;
}

static void command_task(void *argument)
{
    (void)argument;
    raw_command_frame_t frame;
    uint32_t rejected = 0U;
    for (;;) {
        if (xQueueReceive(s_command_queue, &frame, portMAX_DELAY) != pdPASS) continue;
        command_t command;
        const command_parse_result_t parse_result =
            command_parse(frame.data, frame.length, CONFIG_CANSAT_TEAM_ID, &command);
        if (parse_result != COMMAND_PARSE_OK) {
            if (++rejected == 1U || rejected % 20U == 0U) {
                ESP_LOGW(TAG, "rejected commands=%lu (latest parse result %d)",
                         (unsigned long)rejected, (int)parse_result);
            }
            continue;
        }
        rejected = 0U;
        if (apply_command(&command)) ESP_LOGI(TAG, "accepted command: %s", command.echo);
        else ESP_LOGW(TAG, "command not permitted in current mode: %s", command.echo);
    }
}

static void mechanism_task(void *argument)
{
    (void)argument;
    bool request_consumed = false;
    for (;;) {
        uint32_t notification = 0U;
        xTaskNotifyWait(0U, UINT32_MAX, &notification, portMAX_DELAY);
        if (request_consumed) {
            ESP_LOGW(TAG, "mechanism request rejected: one-shot request already consumed");
            if ((notification & MECHANISM_AUTO_BIT) != 0U) {
                const bool success = false;
                xQueueOverwrite(s_mechanism_result_queue, &success);
            }
            continue;
        }

        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        const flight_state_t state = s_flight.state;
        xSemaphoreGive(s_state_mutex);
        const bool automatic_allowed =
            (notification & MECHANISM_AUTO_BIT) != 0U && state == CANSAT_STATE_DESCENT;
#if CONFIG_CANSAT_ALLOW_BENCH_MEC
        const bool bench_allowed = (notification & MECHANISM_BENCH_BIT) != 0U &&
                                   (state == CANSAT_STATE_STARTUP ||
                                    state == CANSAT_STATE_LAUNCH_PAD);
#else
        const bool bench_allowed = false;
#endif
        if (!automatic_allowed && !bench_allowed) {
            ESP_LOGW(TAG, "mechanism request rejected in state %s", cansat_state_name(state));
            continue;
        }

        request_consumed = true;
        esp_err_t result = mechanism_arm();
        if (result == ESP_OK) result = mechanism_fire();
        mechanism_force_safe();
        const bool success = result == ESP_OK;
        ESP_LOGI(TAG, "mechanism request result: %s", esp_err_to_name(result));
        if (automatic_allowed) {
            xQueueOverwrite(s_mechanism_result_queue, &success);
        }
    }
}

static bool parse_gps_time(const char *utc, uint32_t *seconds_of_day)
{
    if (utc == NULL || seconds_of_day == NULL || strlen(utc) < 6U) return false;
    for (size_t i = 0U; i < 6U; ++i) {
        if (utc[i] < '0' || utc[i] > '9') return false;
    }
    const unsigned int hours = (unsigned int)(utc[0] - '0') * 10U + (unsigned int)(utc[1] - '0');
    const unsigned int minutes = (unsigned int)(utc[2] - '0') * 10U + (unsigned int)(utc[3] - '0');
    const unsigned int seconds = (unsigned int)(utc[4] - '0') * 10U + (unsigned int)(utc[5] - '0');
    if (hours >= 24U || minutes >= 60U || seconds >= 60U) return false;
    *seconds_of_day = hours * 3600U + minutes * 60U + seconds;
    return true;
}

static void format_mission_time(char *output, size_t size, const gps_data_t *gps,
                                const control_state_t *control, uint32_t now_ms,
                                mission_clock_t *clock)
{
    uint32_t seconds_of_day;
    if (gps->time_valid && parse_gps_time(gps->utc_time, &seconds_of_day)) {
        clock->gps_synced = true;
        clock->gps_seconds_of_day = seconds_of_day;
        clock->gps_sync_ms = now_ms;
    } else if (clock->gps_synced) {
        seconds_of_day = (clock->gps_seconds_of_day +
                          (now_ms - clock->gps_sync_ms) / 1000U) % 86400U;
    } else {
        seconds_of_day = (now_ms - control->cxon_epoch_ms) / 1000U;
    }
    const unsigned int hours = (seconds_of_day / 3600U) % 100U;
    const unsigned int minutes = (seconds_of_day / 60U) % 60U;
    const unsigned int seconds = seconds_of_day % 60U;
    (void)snprintf(output, size, "%02u:%02u:%02u", hours, minutes, seconds);
}

static void telemetry_task(void *argument)
{
    (void)argument;
    (void)esp_task_wdt_add(NULL);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t packet_count = 0U;
    uint32_t radio_failures = 0U;
    mission_clock_t mission_clock = {0};
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
        (void)esp_task_wdt_reset();
        const uint32_t now = uptime_ms();
        control_state_t control;
        copy_control(&control);
        if (!control.telemetry_enabled) continue;

        telemetry_packet_t packet = {0};
        (void)snprintf(packet.team_id, sizeof(packet.team_id), "%s", CONFIG_CANSAT_TEAM_ID);
        packet.packet_count = packet_count;
        packet.mode = control.simulation_active ? CANSAT_MODE_SIMULATION : CANSAT_MODE_FLIGHT;
        (void)snprintf(packet.cmd_echo, sizeof(packet.cmd_echo), "%s", control.command_echo);
        (void)xQueuePeek(s_sensor_latest_queue, &packet.sensor, 0);

        xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
        packet.gps = s_gps;
        xSemaphoreGive(s_gps_mutex);
        if (packet.gps.time_valid &&
            (now - packet.gps.time_updated_at_ms) > GPS_STALE_MS) packet.gps.time_valid = false;
        if (packet.gps.position_valid &&
            (now - packet.gps.position_updated_at_ms) > GPS_STALE_MS) packet.gps.position_valid = false;
        if (packet.gps.altitude_valid &&
            (now - packet.gps.altitude_updated_at_ms) > GPS_STALE_MS) packet.gps.altitude_valid = false;
        if (packet.gps.satellites_valid &&
            (now - packet.gps.satellites_updated_at_ms) > GPS_STALE_MS) packet.gps.satellites_valid = false;

        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        packet.state = s_flight.state;
        if (s_flight.ground_ready && packet.sensor.altitude_valid) {
            packet.sensor.altitude_m -= s_flight.ground_altitude_m;
        } else {
            packet.sensor.altitude_valid = false;
        }
        xSemaphoreGive(s_state_mutex);

        (void)battery_read(&packet.voltage_v, &packet.voltage_valid,
                           &packet.current_a, &packet.current_valid);
        format_mission_time(packet.mission_time, sizeof(packet.mission_time),
                            &packet.gps, &control, now, &mission_clock);

        char output[CANSAT_TELEMETRY_MAX_LEN];
        size_t length = 0U;
        if (telemetry_format_csv(&packet, output, sizeof(output), &length) !=
            TELEMETRY_FORMAT_OK) {
            ESP_LOGE(TAG, "telemetry formatting failed");
            continue;
        }
        xSemaphoreTake(s_radio_mutex, portMAX_DELAY);
        const esp_err_t result = radio_send(output, length, 500U);
        xSemaphoreGive(s_radio_mutex);
        if (result == ESP_OK) {
            ++packet_count;
            radio_failures = 0U;
        } else if (++radio_failures == 1U || radio_failures % 10U == 0U) {
            ESP_LOGW(TAG, "telemetry send failures=%lu: %s",
                     (unsigned long)radio_failures, esp_err_to_name(result));
        }
    }
}

static void diagnostics_task(void *argument)
{
    (void)argument;
    for (;;) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        const flight_state_t state = s_flight.state;
        xSemaphoreGive(s_state_mutex);
        ESP_LOGI(TAG, "health: free_heap=%lu minimum_free_heap=%lu state=%s",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size(),
                 cansat_state_name(state));
        vTaskDelay(pdMS_TO_TICKS(DIAGNOSTIC_PERIOD_MS));
    }
}

static void create_runtime_objects(void)
{
    s_sensor_latest_queue = xQueueCreate(1U, sizeof(sensor_reading_t));
    s_altitude_queue = xQueueCreate(ALTITUDE_QUEUE_DEPTH, sizeof(sensor_reading_t));
    s_command_queue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(raw_command_frame_t));
    s_mechanism_result_queue = xQueueCreate(1U, sizeof(bool));
    s_gps_mutex = xSemaphoreCreateMutex();
    s_state_mutex = xSemaphoreCreateMutex();
    s_control_mutex = xSemaphoreCreateMutex();
    s_radio_mutex = xSemaphoreCreateMutex();
    configASSERT(s_sensor_latest_queue && s_altitude_queue && s_command_queue &&
                 s_mechanism_result_queue && s_gps_mutex && s_state_mutex &&
                 s_control_mutex && s_radio_mutex);
}

void app_main(void)
{
    const esp_err_t mechanism_result = mechanism_init();
    const bool mechanism_init_failed =
        mechanism_result != ESP_OK && mechanism_result != ESP_ERR_NOT_SUPPORTED;
    if (mechanism_init_failed) {
        ESP_LOGE(TAG, "mechanism safe initialization failed: %s",
                 esp_err_to_name(mechanism_result));
    }
    create_runtime_objects();
    flight_state_config_t state_config = flight_state_default_config();
    state_config.launch_threshold_m = (float)CONFIG_CANSAT_LAUNCH_THRESHOLD_CM / 100.0f;
    state_config.descent_margin_m = (float)CONFIG_CANSAT_DESCENT_MARGIN_CM / 100.0f;
    state_config.landing_band_m = (float)CONFIG_CANSAT_LANDING_BAND_CM / 100.0f;
    state_config.landing_max_step_m = (float)CONFIG_CANSAT_LANDING_MAX_STEP_CM / 100.0f;
    state_config.landing_max_relative_altitude_m =
        (float)CONFIG_CANSAT_LANDING_MAX_HEIGHT_CM / 100.0f;
    configASSERT(flight_state_init(&s_flight, &state_config));
    if (mechanism_init_failed) flight_state_set_error(&s_flight);

    sensors_health_t sensor_health;
    const esp_err_t sensor_result = sensors_init(&sensor_health);
    if (sensor_result != ESP_OK) {
        ESP_LOGE(TAG, "critical barometer initialization failed: %s",
                 esp_err_to_name(sensor_result));
        flight_state_set_error(&s_flight);
    }
    (void)battery_init();
    const esp_err_t gps_result = gps_uart_init();
    const esp_err_t radio_result = radio_init();

    configASSERT(xTaskCreate(mechanism_task, "mechanism", 3072, NULL, 12,
                             &s_mechanism_task_handle) == pdPASS);
    configASSERT(xTaskCreate(flight_state_task, "flight_state", 4096, NULL, 11,
                             NULL) == pdPASS);
    configASSERT(xTaskCreate(command_task, "command", 4096, NULL, 10, NULL) == pdPASS);
    configASSERT(xTaskCreate(sensor_sample_task, "sensors", 4096, NULL, 10, NULL) == pdPASS);
    configASSERT(xTaskCreate(gps_task, "gps", 4096, NULL, 7, NULL) == pdPASS);
    if (gps_result != ESP_OK) {
        ESP_LOGW(TAG, "GPS unavailable: %s", esp_err_to_name(gps_result));
    }
    configASSERT(xTaskCreate(radio_rx_task, "radio_rx", 4096, NULL, 9, NULL) == pdPASS);
    configASSERT(xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 7, NULL) == pdPASS);
    configASSERT(xTaskCreate(diagnostics_task, "diagnostics", 3072, NULL, 3, NULL) == pdPASS);
    if (radio_result != ESP_OK) {
        ESP_LOGE(TAG, "radio unavailable: %s", esp_err_to_name(radio_result));
    }
    ESP_LOGI(TAG, "CanSat startup complete; reset_reason=%d state=%s, telemetry waits for CXON",
             (int)esp_reset_reason(),
             cansat_state_name(s_flight.state));
}
