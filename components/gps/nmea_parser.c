#include "nmea_parser.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NMEA_MAX_SENTENCE 128U
#define NMEA_MAX_FIELDS 24U

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool nmea_checksum_valid(const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$') return false;
    const char *star = strchr(sentence, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0' || star[3] != '\0') return false;
    const int high = hex_value(star[1]);
    const int low = hex_value(star[2]);
    if (high < 0 || low < 0) return false;

    unsigned char checksum = 0U;
    for (const char *p = sentence + 1; p < star; ++p) checksum ^= (unsigned char)*p;
    return checksum == (unsigned char)((high << 4) | low);
}

static bool parse_number(const char *text, double *value)
{
    if (text == NULL || *text == '\0') return false;
    errno = 0;
    char *end = NULL;
    const double parsed = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

static bool parse_unsigned(const char *text, unsigned int maximum,
                           unsigned int *value)
{
    if (text == NULL || *text == '\0' || value == NULL) return false;
    unsigned int parsed = 0U;
    for (const char *p = text; *p != '\0'; ++p) {
        if (!isdigit((unsigned char)*p)) return false;
        const unsigned int digit = (unsigned int)(*p - '0');
        if (parsed > (maximum - digit) / 10U) return false;
        parsed = parsed * 10U + digit;
    }
    *value = parsed;
    return true;
}

static bool parse_time(const char *text, char *output, size_t output_size)
{
    if (text == NULL || strlen(text) < 6U || output == NULL || output_size < 7U) {
        return false;
    }
    for (size_t i = 0U; i < 6U; ++i) {
        if (!isdigit((unsigned char)text[i])) return false;
    }
    if (text[6] != '\0' && text[6] != '.') return false;
    if (text[6] == '.' && text[7] == '\0') return false;
    for (const char *p = text[6] == '.' ? text + 7 : text + 6; *p != '\0'; ++p) {
        if (!isdigit((unsigned char)*p)) return false;
    }
    const unsigned int hours = (unsigned int)(text[0] - '0') * 10U +
                               (unsigned int)(text[1] - '0');
    const unsigned int minutes = (unsigned int)(text[2] - '0') * 10U +
                                 (unsigned int)(text[3] - '0');
    const unsigned int seconds = (unsigned int)(text[4] - '0') * 10U +
                                 (unsigned int)(text[5] - '0');
    if (hours >= 24U || minutes >= 60U || seconds >= 60U) return false;
    (void)snprintf(output, output_size, "%.6s", text);
    return true;
}

static bool parse_coordinate(const char *raw, const char *hemisphere, double *degrees)
{
    double coordinate;
    if (!parse_number(raw, &coordinate) || hemisphere == NULL || hemisphere[0] == '\0' ||
        hemisphere[1] != '\0') {
        return false;
    }
    if (coordinate < 0.0) return false;
    const double whole_degrees = floor(coordinate / 100.0);
    const double minutes = coordinate - whole_degrees * 100.0;
    if (minutes < 0.0 || minutes >= 60.0) return false;
    double result = whole_degrees + minutes / 60.0;
    double limit;
    if (hemisphere[0] == 'N' || hemisphere[0] == 'S') limit = 90.0;
    else if (hemisphere[0] == 'E' || hemisphere[0] == 'W') limit = 180.0;
    else return false;
    if (result > limit) return false;
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') result = -result;
    *degrees = result;
    return true;
}

static bool supported_type(const char *type, const char *suffix)
{
    const size_t len = strlen(type);
    return len == 5U && strcmp(type + 2U, suffix) == 0;
}

bool nmea_parse_sentence(const char *sentence, uint32_t timestamp_ms,
                         gps_data_t *gps, uint32_t *updates)
{
    if (updates != NULL) *updates = NMEA_UPDATE_NONE;
    if (sentence == NULL || gps == NULL || !nmea_checksum_valid(sentence)) return false;
    const char *star = strchr(sentence, '*');
    const size_t body_len = (size_t)(star - (sentence + 1));
    if (body_len == 0U || body_len >= NMEA_MAX_SENTENCE) return false;

    char body[NMEA_MAX_SENTENCE];
    memcpy(body, sentence + 1, body_len);
    body[body_len] = '\0';
    char *fields[NMEA_MAX_FIELDS] = {0};
    size_t count = 0U;
    fields[count++] = body;
    for (char *p = body; *p != '\0' && count < NMEA_MAX_FIELDS; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }

    uint32_t changed = NMEA_UPDATE_NONE;
    if (supported_type(fields[0], "GGA")) {
        if (count < 10U) return false;
        unsigned int quality = 0U;
        if (!parse_unsigned(fields[6], 8U, &quality)) return false;
        const bool valid = quality > 0U;
        gps->fix_valid = valid;
        if (parse_time(fields[1], gps->utc_time, sizeof(gps->utc_time))) {
            gps->time_valid = true;
            gps->time_updated_at_ms = timestamp_ms;
            changed |= NMEA_UPDATE_TIME;
        }
        if (valid) {
            double lat, lon, alt;
            if (parse_coordinate(fields[2], fields[3], &lat) &&
                parse_coordinate(fields[4], fields[5], &lon)) {
                gps->latitude_deg = lat;
                gps->longitude_deg = lon;
                gps->position_valid = true;
                gps->position_updated_at_ms = timestamp_ms;
                changed |= NMEA_UPDATE_POSITION;
            } else {
                gps->position_valid = false;
            }
            if (parse_number(fields[9], &alt) &&
                (fields[10][0] == '\0' || strcmp(fields[10], "M") == 0)) {
                gps->altitude_m = (float)alt;
                gps->altitude_valid = true;
                gps->altitude_updated_at_ms = timestamp_ms;
                changed |= NMEA_UPDATE_ALTITUDE;
            } else {
                gps->altitude_valid = false;
            }
            unsigned int satellites;
            if (parse_unsigned(fields[7], 255U, &satellites)) {
                gps->satellites = (uint8_t)satellites;
                gps->satellites_valid = true;
                gps->satellites_updated_at_ms = timestamp_ms;
                changed |= NMEA_UPDATE_SATELLITES;
            } else {
                gps->satellites_valid = false;
            }
        } else {
            gps->position_valid = false;
            gps->altitude_valid = false;
            gps->satellites_valid = false;
        }
    } else if (supported_type(fields[0], "RMC")) {
        if (count < 7U) return false;
        const bool valid = strcmp(fields[2], "A") == 0;
        gps->fix_valid = valid;
        if (parse_time(fields[1], gps->utc_time, sizeof(gps->utc_time))) {
            gps->time_valid = true;
            gps->time_updated_at_ms = timestamp_ms;
            changed |= NMEA_UPDATE_TIME;
        }
        if (valid) {
            double lat, lon;
            if (parse_coordinate(fields[3], fields[4], &lat) &&
                parse_coordinate(fields[5], fields[6], &lon)) {
                gps->latitude_deg = lat;
                gps->longitude_deg = lon;
                gps->position_valid = true;
                gps->position_updated_at_ms = timestamp_ms;
                changed |= NMEA_UPDATE_POSITION;
            } else {
                gps->position_valid = false;
            }
        } else {
            gps->position_valid = false;
        }
    } else {
        return false;
    }

    if (updates != NULL) *updates = changed;
    return true;
}
