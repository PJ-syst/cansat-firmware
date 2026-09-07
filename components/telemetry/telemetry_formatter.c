#include "telemetry_formatter.h"

#include <math.h>
#include <stdio.h>

static void format_float(char *output, size_t size, float value, bool valid,
                         unsigned int precision)
{
    if (!valid || !isfinite(value)) {
        (void)snprintf(output, size, "NA");
        return;
    }
    (void)snprintf(output, size, "%.*f", (int)precision, (double)value);
}

static void format_double(char *output, size_t size, double value, bool valid,
                          unsigned int precision)
{
    if (!valid || !isfinite(value)) {
        (void)snprintf(output, size, "NA");
        return;
    }
    (void)snprintf(output, size, "%.*f", (int)precision, value);
}

static void sanitize_echo(const char *input, char *output, size_t output_size)
{
    size_t i = 0U;
    for (; input != NULL && input[i] != '\0' && i + 1U < output_size; ++i) {
        const char c = input[i];
        output[i] = (c == ',' || c == '\r' || c == '\n') ? '_' : c;
    }
    output[i] = '\0';
}

telemetry_format_result_t telemetry_format_csv(const telemetry_packet_t *packet,
                                               char *output,
                                               size_t output_size,
                                               size_t *written)
{
    if (packet == NULL || output == NULL || output_size == 0U) {
        return TELEMETRY_FORMAT_INVALID_ARGUMENT;
    }

    char altitude[24], temperature[24], pressure[24], voltage[24], current[24];
    char gyro[3][24], accel[3][24], gps_altitude[24], latitude[32], longitude[32];
    char satellites[8];
    char echo[CANSAT_CMD_ECHO_MAX_LEN];
    format_float(altitude, sizeof(altitude), packet->sensor.altitude_m,
                 packet->sensor.altitude_valid, 1U);
    format_float(temperature, sizeof(temperature), packet->sensor.temperature_c,
                 packet->sensor.temperature_valid, 1U);
    format_float(pressure, sizeof(pressure), packet->sensor.pressure_kpa,
                 packet->sensor.pressure_valid, 2U);
    format_float(voltage, sizeof(voltage), packet->voltage_v, packet->voltage_valid, 2U);
    format_float(current, sizeof(current), packet->current_a, packet->current_valid, 2U);
    for (size_t i = 0U; i < 3U; ++i) {
        format_float(gyro[i], sizeof(gyro[i]), packet->sensor.gyro_dps[i],
                     packet->sensor.imu_valid, 2U);
        format_float(accel[i], sizeof(accel[i]), packet->sensor.accel_g[i],
                     packet->sensor.imu_valid, 2U);
    }
    format_float(gps_altitude, sizeof(gps_altitude), packet->gps.altitude_m,
                  packet->gps.altitude_valid, 1U);
    format_double(latitude, sizeof(latitude), packet->gps.latitude_deg,
                   packet->gps.position_valid, 6U);
    format_double(longitude, sizeof(longitude), packet->gps.longitude_deg,
                   packet->gps.position_valid, 6U);
    if (packet->gps.satellites_valid) {
        (void)snprintf(satellites, sizeof(satellites), "%u",
                       (unsigned int)packet->gps.satellites);
    } else {
        (void)snprintf(satellites, sizeof(satellites), "NA");
    }
    sanitize_echo(packet->cmd_echo, echo, sizeof(echo));

    const int count = snprintf(
        output, output_size,
        "%s,%s,%lu,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\r",
        packet->team_id, packet->mission_time, (unsigned long)packet->packet_count,
        cansat_mode_name(packet->mode), cansat_state_name(packet->state), altitude,
        temperature, pressure, voltage, current, gyro[0], gyro[1], gyro[2],
        accel[0], accel[1], accel[2],
        packet->gps.time_valid ? packet->gps.utc_time : "NA", gps_altitude,
        latitude, longitude, satellites, echo);

    if (count < 0 || (size_t)count >= output_size) {
        output[output_size - 1U] = '\0';
        return TELEMETRY_FORMAT_TRUNCATED;
    }
    if (written != NULL) {
        *written = (size_t)count;
    }
    return TELEMETRY_FORMAT_OK;
}
