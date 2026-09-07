#include "command_parser.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void trim_frame(char *frame)
{
    size_t len = strlen(frame);
    while (len > 0U && isspace((unsigned char)frame[len - 1U])) {
        frame[--len] = '\0';
    }

    size_t start = 0U;
    while (frame[start] != '\0' && isspace((unsigned char)frame[start])) {
        ++start;
    }
    if (start > 0U) {
        memmove(frame, frame + start, strlen(frame + start) + 1U);
    }
}

static void set_echo(command_t *command, const char *frame)
{
    size_t out = 0U;
    while (frame[out] != '\0' && out + 1U < sizeof(command->echo)) {
        const char c = frame[out];
        command->echo[out] = (c == ',' || c == '\r' || c == '\n') ? '_' : c;
        ++out;
    }
    command->echo[out] = '\0';
}

static command_parse_result_t parse_pressure(const char *value, command_t *command)
{
    if (value == NULL || *value == '\0') {
        return COMMAND_PARSE_BAD_VALUE;
    }

    errno = 0;
    char *end = NULL;
    const float pressure = strtof(value, &end);
    while (end != NULL && isspace((unsigned char)*end)) {
        ++end;
    }
    if (errno != 0 || end == value || end == NULL || *end != '\0' ||
        !isfinite(pressure) || pressure < 30.0f || pressure > 120.0f) {
        return COMMAND_PARSE_BAD_VALUE;
    }

    command->type = CANSAT_CMD_SIM_PRESSURE;
    command->pressure_kpa = pressure;
    return COMMAND_PARSE_OK;
}

static bool valid_team_id(const char *team_id)
{
    if (team_id == NULL || *team_id == '\0' ||
        strlen(team_id) >= CANSAT_TEAM_ID_MAX_LEN) {
        return false;
    }
    for (const char *p = team_id; *p != '\0'; ++p) {
        if (!isalnum((unsigned char)*p) && *p != '_' && *p != '-') return false;
    }
    return true;
}

command_parse_result_t command_parse(const char *input, size_t input_len,
                                     const char *expected_team_id,
                                     command_t *command)
{
    if (input == NULL || command == NULL || input_len == 0U) {
        return COMMAND_PARSE_EMPTY;
    }
    if (input_len >= CANSAT_COMMAND_MAX_LEN) {
        return COMMAND_PARSE_TOO_LONG;
    }

    char frame[CANSAT_COMMAND_MAX_LEN];
    memcpy(frame, input, input_len);
    frame[input_len] = '\0';
    trim_frame(frame);
    if (frame[0] == '\0') {
        return COMMAND_PARSE_EMPTY;
    }

    memset(command, 0, sizeof(*command));
    set_echo(command, frame);

    char *separator = strchr(frame, ',');
    if (separator == NULL) return COMMAND_PARSE_BAD_TEAM;
    *separator = '\0';
    const char *payload = separator + 1;
    if (!valid_team_id(frame) || expected_team_id == NULL ||
        strcmp(frame, expected_team_id) != 0) {
        return COMMAND_PARSE_BAD_TEAM;
    }
    const size_t team_length = strlen(frame);
    memcpy(command->team_id, frame, team_length + 1U);
    if (*payload == '\0') return COMMAND_PARSE_EMPTY;

    if (strcmp(payload, "CXON") == 0) {
        command->type = CANSAT_CMD_CXON;
        return COMMAND_PARSE_OK;
    }
    if (strcmp(payload, "CXOFF") == 0) {
        command->type = CANSAT_CMD_CXOFF;
        return COMMAND_PARSE_OK;
    }
    if (strcmp(payload, "SIM_ENABLE") == 0) {
        command->type = CANSAT_CMD_SIM_ENABLE;
        return COMMAND_PARSE_OK;
    }
    if (strcmp(payload, "SIM_ACTIVATE") == 0) {
        command->type = CANSAT_CMD_SIM_ACTIVATE;
        return COMMAND_PARSE_OK;
    }
    if (strcmp(payload, "SIM_DISABLE") == 0) {
        command->type = CANSAT_CMD_SIM_DISABLE;
        return COMMAND_PARSE_OK;
    }
    if (strncmp(payload, "SP", 2U) == 0) {
        const char *value = payload + 2U;
        if (*value == ',' || *value == '=') {
            ++value;
        }
        return parse_pressure(value, command);
    }
    if (strncmp(payload, "SIM_PRESSURE,", 13U) == 0) {
        return parse_pressure(payload + 13U, command);
    }
    if (strcmp(payload, "MEC") == 0 || strncmp(payload, "MEC,", 4U) == 0) {
        command->type = CANSAT_CMD_MEC;
        const char *name = payload[3] == ',' ? payload + 4U : "PRIMARY";
        if (*name == '\0' || strlen(name) >= sizeof(command->mechanism)) {
            return COMMAND_PARSE_BAD_VALUE;
        }
        for (const char *p = name; *p != '\0'; ++p) {
            if (!isalnum((unsigned char)*p) && *p != '_') {
                return COMMAND_PARSE_BAD_VALUE;
            }
        }
        (void)snprintf(command->mechanism, sizeof(command->mechanism), "%s", name);
        return COMMAND_PARSE_OK;
    }

    return COMMAND_PARSE_UNKNOWN;
}
