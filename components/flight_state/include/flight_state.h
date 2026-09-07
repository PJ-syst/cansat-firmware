#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "cansat_types.h"

#define CANSAT_STATE_HISTORY_MAX 16U

typedef struct {
    uint8_t ground_sample_count;
    uint8_t trend_window;
    uint8_t launch_confirmations;
    uint8_t descent_confirmations;
    uint16_t landing_confirmations;
    float ground_stability_band_m;
    float launch_threshold_m;
    float descent_margin_m;
    float landing_band_m;
    float landing_max_step_m;
    float landing_max_relative_altitude_m;
} flight_state_config_t;

typedef enum {
    FLIGHT_EVENT_NONE = 0,
    FLIGHT_EVENT_STATE_CHANGED = 1 << 0,
    FLIGHT_EVENT_REQUEST_MECHANISM = 1 << 1,
} flight_event_t;

typedef struct {
    flight_state_config_t config;
    flight_state_t state;
    float ground_altitude_m;
    float relative_altitude_m;
    float ground_sum;
    float ground_min;
    float ground_max;
    float history[CANSAT_STATE_HISTORY_MAX];
    float landing_anchor_m;
    float landing_previous_m;
    uint16_t ground_count;
    uint16_t history_count;
    uint16_t history_head;
    uint16_t launch_count;
    uint16_t descent_count;
    uint16_t landing_count;
    bool ground_ready;
    bool mechanism_requested;
    bool mechanism_fired;
    bool landing_previous_valid;
} flight_state_machine_t;

flight_state_config_t flight_state_default_config(void);
bool flight_state_init(flight_state_machine_t *machine,
                       const flight_state_config_t *config);
flight_event_t flight_state_process(flight_state_machine_t *machine,
                                    float absolute_altitude_m,
                                    bool sample_valid);
flight_event_t flight_state_report_mechanism(flight_state_machine_t *machine,
                                             bool success);
void flight_state_set_error(flight_state_machine_t *machine);

#endif
