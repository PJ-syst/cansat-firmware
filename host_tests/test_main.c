#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cansat_types.h"
#include "command_parser.h"
#include "flight_state.h"
#include "nmea_parser.h"
#include "telemetry_formatter.h"

static void test_commands(void)
{
    command_t command;
    const char *cxon = "TEAM1,CXON\r\n";
    assert(command_parse(cxon, strlen(cxon), "TEAM1", &command) == COMMAND_PARSE_OK);
    assert(command.type == CANSAT_CMD_CXON);
    assert(strcmp(command.team_id, "TEAM1") == 0);
    const char *pressure = "TEAM1,SP=98.75";
    assert(command_parse(pressure, strlen(pressure), "TEAM1", &command) == COMMAND_PARSE_OK);
    assert(command.type == CANSAT_CMD_SIM_PRESSURE);
    assert(fabsf(command.pressure_kpa - 98.75f) < 0.001f);
    assert(command_parse("TEAM1,SP=999", 12U, "TEAM1", &command) == COMMAND_PARSE_BAD_VALUE);
    assert(command_parse("TEAM1,MEC,PRIMARY", 17U, "TEAM1", &command) == COMMAND_PARSE_OK);
    assert(strcmp(command.mechanism, "PRIMARY") == 0);
    assert(command_parse("TEAM1,MEC,BAD,NAME", 18U, "TEAM1", &command) == COMMAND_PARSE_BAD_VALUE);
    assert(command_parse("OTHER,CXON", 10U, "TEAM1", &command) == COMMAND_PARSE_BAD_TEAM);
    assert(command_parse("CXON", 4U, "TEAM1", &command) == COMMAND_PARSE_BAD_TEAM);
    assert(command_parse("TEAM1,SIM_DISABLE", 17U, "TEAM1", &command) == COMMAND_PARSE_OK);
    assert(command.type == CANSAT_CMD_SIM_DISABLE);
    assert(command_parse("TEAM1,unknown", 13U, "TEAM1", &command) == COMMAND_PARSE_UNKNOWN);
}

static void test_nmea(void)
{
    static const char gga[] =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    gps_data_t gps = {0};
    uint32_t updates = 0U;
    assert(nmea_checksum_valid(gga));
    assert(nmea_parse_sentence(gga, 1234U, &gps, &updates));
    assert(gps.fix_valid);
    assert(gps.time_valid && gps.position_valid && gps.altitude_valid &&
           gps.satellites_valid);
    assert(gps.satellites == 8U);
    assert(fabs(gps.latitude_deg - 48.1173) < 0.000001);
    assert(fabs(gps.longitude_deg - 11.5166667) < 0.000001);
    assert(fabsf(gps.altitude_m - 545.4f) < 0.01f);
    assert(strcmp(gps.utc_time, "123519") == 0);
    assert((updates & NMEA_UPDATE_POSITION) != 0U);
    assert(!nmea_checksum_valid("$GPGGA,broken*00"));
    assert(!nmea_checksum_valid("$GPGGA,broken*00EXTRA"));

    static const char rmc[] =
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    gps_data_t rmc_only = {0};
    assert(nmea_parse_sentence(rmc, 2000U, &rmc_only, NULL));
    assert(rmc_only.fix_valid && rmc_only.time_valid && rmc_only.position_valid);
    assert(!rmc_only.altitude_valid && !rmc_only.satellites_valid);
}

static void test_telemetry(void)
{
    telemetry_packet_t packet = {0};
    (void)snprintf(packet.team_id, sizeof(packet.team_id), "TEAM1");
    (void)snprintf(packet.mission_time, sizeof(packet.mission_time), "00:00:01");
    (void)snprintf(packet.gps.utc_time, sizeof(packet.gps.utc_time), "123519");
    (void)snprintf(packet.cmd_echo, sizeof(packet.cmd_echo), "SP,98.75");
    packet.packet_count = 1U;
    packet.mode = CANSAT_MODE_FLIGHT;
    packet.state = CANSAT_STATE_ASCENT;
    packet.sensor = (sensor_reading_t){
        .altitude_m = 12.34f,
        .temperature_c = 24.5f,
        .pressure_kpa = 99.12f,
        .gyro_dps = {1.0f, 2.0f, 3.0f},
        .accel_g = {0.1f, 0.2f, 0.9f},
        .barometer_valid = true,
        .altitude_valid = true,
        .temperature_valid = true,
        .pressure_valid = true,
        .imu_valid = true,
    };
    packet.gps.fix_valid = true;
    packet.gps.time_valid = true;
    packet.gps.position_valid = true;
    packet.gps.altitude_valid = true;
    packet.gps.satellites_valid = true;
    packet.gps.altitude_m = 550.0f;
    packet.gps.latitude_deg = 0.347596;
    packet.gps.longitude_deg = 32.582520;
    packet.gps.satellites = 9U;
    packet.voltage_v = 4.12f;
    packet.current_a = 0.25f;
    packet.voltage_valid = true;
    packet.current_valid = true;

    char output[CANSAT_TELEMETRY_MAX_LEN];
    size_t written = 0U;
    assert(telemetry_format_csv(&packet, output, sizeof(output), &written) ==
           TELEMETRY_FORMAT_OK);
    assert(written == strlen(output));
    assert(output[written - 1U] == '\r');
    unsigned int commas = 0U;
    for (size_t i = 0U; i < written; ++i) {
        if (output[i] == ',') ++commas;
    }
    assert(commas == 21U);
    assert(strstr(output, "SP_98.75\r") != NULL);

    char small[16];
    assert(telemetry_format_csv(&packet, small, sizeof(small), NULL) ==
           TELEMETRY_FORMAT_TRUNCATED);
}

static void test_flight_state(void)
{
    flight_state_config_t config = flight_state_default_config();
    config.ground_sample_count = 3U;
    config.trend_window = 3U;
    config.launch_confirmations = 2U;
    config.descent_confirmations = 2U;
    config.landing_confirmations = 3U;

    flight_state_machine_t machine;
    assert(flight_state_init(&machine, &config));
    assert(flight_state_process(&machine, 100.0f, true) == FLIGHT_EVENT_NONE);
    assert(flight_state_process(&machine, 100.1f, true) == FLIGHT_EVENT_NONE);
    assert((flight_state_process(&machine, 99.9f, true) & FLIGHT_EVENT_STATE_CHANGED) != 0U);
    assert(machine.state == CANSAT_STATE_LAUNCH_PAD);

    (void)flight_state_process(&machine, 111.0f, true);
    assert((flight_state_process(&machine, 112.0f, true) & FLIGHT_EVENT_STATE_CHANGED) != 0U);
    assert(machine.state == CANSAT_STATE_ASCENT);

    (void)flight_state_process(&machine, 120.0f, true);
    (void)flight_state_process(&machine, 118.0f, true);
    (void)flight_state_process(&machine, 116.0f, true);
    const flight_event_t descent_event = flight_state_process(&machine, 114.0f, true);
    assert((descent_event & FLIGHT_EVENT_REQUEST_MECHANISM) != 0U);
    assert(machine.state == CANSAT_STATE_DESCENT);

    assert((flight_state_report_mechanism(&machine, true) & FLIGHT_EVENT_STATE_CHANGED) != 0U);
    assert(machine.state == CANSAT_STATE_MECH_RELEASED);
    assert(flight_state_report_mechanism(&machine, true) == FLIGHT_EVENT_NONE);

    for (int i = 0; i < 10; ++i) {
        (void)flight_state_process(&machine, 120.0f, true);
    }
    assert(machine.state == CANSAT_STATE_MECH_RELEASED);

    (void)flight_state_process(&machine, 100.1f, true);
    (void)flight_state_process(&machine, 100.2f, true);
    (void)flight_state_process(&machine, 100.15f, true);
    assert((flight_state_process(&machine, 100.1f, true) & FLIGHT_EVENT_STATE_CHANGED) != 0U);
    assert(machine.state == CANSAT_STATE_LANDED);

    flight_state_machine_t failed;
    assert(flight_state_init(&failed, &config));
    failed.state = CANSAT_STATE_DESCENT;
    assert((flight_state_report_mechanism(&failed, false) & FLIGHT_EVENT_STATE_CHANGED) != 0U);
    assert(failed.state == CANSAT_STATE_ERROR);
}

int main(void)
{
    test_commands();
    test_nmea();
    test_telemetry();
    test_flight_state();
    puts("All CanSat host tests passed.");
    return 0;
}
