#ifndef TELEMETRY_FORMATTER_H
#define TELEMETRY_FORMATTER_H

#include <stddef.h>

#include "cansat_types.h"

typedef enum {
    TELEMETRY_FORMAT_OK = 0,
    TELEMETRY_FORMAT_INVALID_ARGUMENT,
    TELEMETRY_FORMAT_TRUNCATED,
} telemetry_format_result_t;

telemetry_format_result_t telemetry_format_csv(const telemetry_packet_t *packet,
                                               char *output,
                                               size_t output_size,
                                               size_t *written);

#endif
