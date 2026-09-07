#ifndef CANSAT_TYPES_H
#define CANSAT_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CANSAT_COMMAND_MAX_LEN 96U
#define CANSAT_CMD_ECHO_MAX_LEN 48U
#define CANSAT_MISSION_TIME_MAX_LEN 16U
#define CANSAT_GPS_TIME_MAX_LEN 16U
#define CANSAT_TEAM_ID_MAX_LEN 16U
#define CANSAT_MECHANISM_NAME_MAX_LEN 16U
#define CANSAT_TELEMETRY_MAX_LEN 320U

typedef enum {
    CANSAT_STATE_STARTUP = 0,
    CANSAT_STATE_LAUNCH_PAD,
    CANSAT_STATE_ASCENT,
    CANSAT_STATE_DESCENT,
    CANSAT_STATE_MECH_RELEASED,
    CANSAT_STATE_LANDED,
    CANSAT_STATE_ERROR,
} flight_state_t;

typedef enum {
    CANSAT_MODE_FLIGHT = 0,
    CANSAT_MODE_SIMULATION,
} cansat_mode_t;

typedef enum {
    CANSAT_CMD_INVALID = 0,
    CANSAT_CMD_CXON,
    CANSAT_CMD_CXOFF,
    CANSAT_CMD_SIM_ENABLE,
    CANSAT_CMD_SIM_ACTIVATE,
    CANSAT_CMD_SIM_DISABLE,
    CANSAT_CMD_SIM_PRESSURE,
    CANSAT_CMD_MEC,
} command_type_t;

typedef struct {
    float altitude_m;
    float temperature_c;
    float pressure_kpa;
    float gyro_dps[3];
    float accel_g[3];
    uint32_t timestamp_ms;
    bool barometer_valid;
    bool altitude_valid;
    bool temperature_valid;
    bool pressure_valid;
    bool imu_valid;
} sensor_reading_t;

typedef struct {
    char utc_time[CANSAT_GPS_TIME_MAX_LEN];
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    uint32_t time_updated_at_ms;
    uint32_t position_updated_at_ms;
    uint32_t altitude_updated_at_ms;
    uint32_t satellites_updated_at_ms;
    uint8_t satellites;
    bool fix_valid;
    bool time_valid;
    bool position_valid;
    bool altitude_valid;
    bool satellites_valid;
} gps_data_t;

typedef struct {
    command_type_t type;
    float pressure_kpa;
    char team_id[CANSAT_TEAM_ID_MAX_LEN];
    char mechanism[CANSAT_MECHANISM_NAME_MAX_LEN];
    char echo[CANSAT_CMD_ECHO_MAX_LEN];
} command_t;

typedef struct {
    char team_id[CANSAT_TEAM_ID_MAX_LEN];
    char mission_time[CANSAT_MISSION_TIME_MAX_LEN];
    uint32_t packet_count;
    cansat_mode_t mode;
    flight_state_t state;
    sensor_reading_t sensor;
    gps_data_t gps;
    float voltage_v;
    float current_a;
    bool voltage_valid;
    bool current_valid;
    char cmd_echo[CANSAT_CMD_ECHO_MAX_LEN];
} telemetry_packet_t;

const char *cansat_state_name(flight_state_t state);
const char *cansat_mode_name(cansat_mode_t mode);

#endif
