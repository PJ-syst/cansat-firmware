#ifndef CANSAT_RADIO_H
#define CANSAT_RADIO_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t radio_init(void);
esp_err_t radio_recover(void);
esp_err_t radio_send(const char *data, size_t length, uint32_t timeout_ms);
int radio_read(uint8_t *buffer, size_t capacity, uint32_t timeout_ms);

#endif
