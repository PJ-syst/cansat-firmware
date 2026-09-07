#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdbool.h>
#include <stdint.h>

#include "cansat_types.h"

typedef enum {
    NMEA_UPDATE_NONE = 0,
    NMEA_UPDATE_TIME = 1 << 0,
    NMEA_UPDATE_POSITION = 1 << 1,
    NMEA_UPDATE_ALTITUDE = 1 << 2,
    NMEA_UPDATE_SATELLITES = 1 << 3,
} nmea_update_t;

bool nmea_checksum_valid(const char *sentence);
bool nmea_parse_sentence(const char *sentence, uint32_t timestamp_ms,
                         gps_data_t *gps, uint32_t *updates);

#endif
