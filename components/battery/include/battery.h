#ifndef CANSAT_BATTERY_H
#define CANSAT_BATTERY_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t battery_init(void);
esp_err_t battery_read(float *voltage_v, bool *voltage_valid,
                       float *current_a, bool *current_valid);

#endif
