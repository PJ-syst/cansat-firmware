#ifndef CANSAT_GPS_UART_H
#define CANSAT_GPS_UART_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t gps_uart_init(void);
esp_err_t gps_uart_recover(void);
int gps_uart_read(uint8_t *buffer, size_t capacity, uint32_t timeout_ms);

#endif
