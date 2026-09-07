#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stddef.h>

#include "cansat_types.h"

typedef enum {
    COMMAND_PARSE_OK = 0,
    COMMAND_PARSE_EMPTY,
    COMMAND_PARSE_TOO_LONG,
    COMMAND_PARSE_UNKNOWN,
    COMMAND_PARSE_BAD_VALUE,
    COMMAND_PARSE_BAD_TEAM,
} command_parse_result_t;

command_parse_result_t command_parse(const char *input, size_t input_len,
                                     const char *expected_team_id,
                                     command_t *command);

#endif
