#include "flight_state.h"

#include <float.h>
#include <math.h>
#include <string.h>

flight_state_config_t flight_state_default_config(void)
{
    return (flight_state_config_t){
        .ground_sample_count = 20U,
        .trend_window = 5U,
        .launch_confirmations = 3U,
        .descent_confirmations = 3U,
        .landing_confirmations = 50U,
        .ground_stability_band_m = 2.0f,
        .launch_threshold_m = 10.0f,
        .descent_margin_m = 1.5f,
        .landing_band_m = 2.0f,
        .landing_max_step_m = 0.25f,
        .landing_max_relative_altitude_m = 5.0f,
    };
}

static bool config_valid(const flight_state_config_t *config)
{
    return config != NULL && config->ground_sample_count >= 2U &&
           config->trend_window >= 2U &&
           config->trend_window <= CANSAT_STATE_HISTORY_MAX &&
           config->launch_confirmations > 0U &&
           config->descent_confirmations > 0U &&
           config->landing_confirmations > 0U &&
           config->ground_stability_band_m > 0.0f &&
           config->launch_threshold_m > 0.0f &&
           config->descent_margin_m > 0.0f &&
           config->landing_band_m > 0.0f &&
           config->landing_max_step_m > 0.0f &&
           config->landing_max_relative_altitude_m > 0.0f;
}

bool flight_state_init(flight_state_machine_t *machine,
                       const flight_state_config_t *config)
{
    if (machine == NULL || !config_valid(config)) {
        return false;
    }
    memset(machine, 0, sizeof(*machine));
    machine->config = *config;
    machine->state = CANSAT_STATE_STARTUP;
    machine->ground_min = FLT_MAX;
    machine->ground_max = -FLT_MAX;
    return true;
}

static void reset_ground_window(flight_state_machine_t *machine, float altitude)
{
    machine->ground_sum = altitude;
    machine->ground_min = altitude;
    machine->ground_max = altitude;
    machine->ground_count = 1U;
}

static flight_event_t calibrate_ground(flight_state_machine_t *machine,
                                       float altitude)
{
    if (machine->ground_count == 0U) {
        reset_ground_window(machine, altitude);
        return FLIGHT_EVENT_NONE;
    }

    const float next_min = fminf(machine->ground_min, altitude);
    const float next_max = fmaxf(machine->ground_max, altitude);
    if ((next_max - next_min) > machine->config.ground_stability_band_m) {
        reset_ground_window(machine, altitude);
        return FLIGHT_EVENT_NONE;
    }

    machine->ground_min = next_min;
    machine->ground_max = next_max;
    machine->ground_sum += altitude;
    ++machine->ground_count;
    if (machine->ground_count >= machine->config.ground_sample_count) {
        machine->ground_altitude_m = machine->ground_sum / (float)machine->ground_count;
        machine->ground_ready = true;
        machine->relative_altitude_m = altitude - machine->ground_altitude_m;
        machine->state = CANSAT_STATE_LAUNCH_PAD;
        return FLIGHT_EVENT_STATE_CHANGED;
    }
    return FLIGHT_EVENT_NONE;
}

static void push_history(flight_state_machine_t *machine, float altitude)
{
    machine->history[machine->history_head] = altitude;
    machine->history_head = (machine->history_head + 1U) % machine->config.trend_window;
    if (machine->history_count < machine->config.trend_window) {
        ++machine->history_count;
    }
}

static bool descending(const flight_state_machine_t *machine)
{
    if (machine->history_count < machine->config.trend_window) {
        return false;
    }
    const uint16_t oldest_index = machine->history_head;
    const uint16_t newest_index = (machine->history_head + machine->config.trend_window - 1U) %
                                  machine->config.trend_window;
    return (machine->history[oldest_index] - machine->history[newest_index]) >
           machine->config.descent_margin_m;
}

static bool landing_stable(flight_state_machine_t *machine, float altitude)
{
    const bool near_ground = fabsf(altitude) <=
                             machine->config.landing_max_relative_altitude_m;
    const bool moving_slowly = !machine->landing_previous_valid ||
                               fabsf(altitude - machine->landing_previous_m) <=
                                   machine->config.landing_max_step_m;
    machine->landing_previous_m = altitude;
    machine->landing_previous_valid = true;
    if (!near_ground || !moving_slowly) {
        machine->landing_count = 0U;
        machine->landing_anchor_m = altitude;
        return false;
    }
    if (machine->landing_count == 0U) {
        machine->landing_anchor_m = altitude;
        machine->landing_count = 1U;
        return false;
    }
    if (fabsf(altitude - machine->landing_anchor_m) <= machine->config.landing_band_m) {
        ++machine->landing_count;
    } else {
        machine->landing_anchor_m = altitude;
        machine->landing_count = 1U;
    }
    return machine->landing_count >= machine->config.landing_confirmations;
}

flight_event_t flight_state_process(flight_state_machine_t *machine,
                                    float absolute_altitude_m,
                                    bool sample_valid)
{
    if (machine == NULL || !sample_valid || !isfinite(absolute_altitude_m) ||
        machine->state == CANSAT_STATE_ERROR || machine->state == CANSAT_STATE_LANDED) {
        return FLIGHT_EVENT_NONE;
    }
    if (machine->state == CANSAT_STATE_STARTUP) {
        return calibrate_ground(machine, absolute_altitude_m);
    }

    machine->relative_altitude_m = absolute_altitude_m - machine->ground_altitude_m;
    push_history(machine, machine->relative_altitude_m);

    if (machine->state == CANSAT_STATE_LAUNCH_PAD) {
        if (machine->relative_altitude_m >= machine->config.launch_threshold_m) {
            ++machine->launch_count;
        } else {
            machine->launch_count = 0U;
        }
        if (machine->launch_count >= machine->config.launch_confirmations) {
            machine->state = CANSAT_STATE_ASCENT;
            machine->history_count = 0U;
            machine->history_head = 0U;
            return FLIGHT_EVENT_STATE_CHANGED;
        }
    } else if (machine->state == CANSAT_STATE_ASCENT) {
        machine->descent_count = descending(machine) ? machine->descent_count + 1U : 0U;
        if (machine->descent_count >= machine->config.descent_confirmations) {
            machine->state = CANSAT_STATE_DESCENT;
            machine->mechanism_requested = true;
            machine->landing_count = 0U;
            machine->landing_previous_valid = false;
            return FLIGHT_EVENT_STATE_CHANGED | FLIGHT_EVENT_REQUEST_MECHANISM;
        }
    } else if (machine->state == CANSAT_STATE_DESCENT ||
               machine->state == CANSAT_STATE_MECH_RELEASED) {
        if (landing_stable(machine, machine->relative_altitude_m)) {
            machine->state = CANSAT_STATE_LANDED;
            return FLIGHT_EVENT_STATE_CHANGED;
        }
    }
    return FLIGHT_EVENT_NONE;
}

flight_event_t flight_state_report_mechanism(flight_state_machine_t *machine,
                                             bool success)
{
    if (machine == NULL || machine->mechanism_fired ||
        machine->state != CANSAT_STATE_DESCENT) {
        return FLIGHT_EVENT_NONE;
    }
    if (!success) {
        machine->state = CANSAT_STATE_ERROR;
        return FLIGHT_EVENT_STATE_CHANGED;
    }
    machine->mechanism_fired = true;
    machine->state = CANSAT_STATE_MECH_RELEASED;
    return FLIGHT_EVENT_STATE_CHANGED;
}

void flight_state_set_error(flight_state_machine_t *machine)
{
    if (machine != NULL && machine->state != CANSAT_STATE_LANDED) {
        machine->state = CANSAT_STATE_ERROR;
    }
}
