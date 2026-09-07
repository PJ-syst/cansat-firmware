#include "gps_uart.h"

#include <stdbool.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

static bool s_driver_installed;
static bool s_ready;

esp_err_t gps_uart_init(void)
{
    if (s_ready) return ESP_OK;
    const uart_port_t port = (uart_port_t)CONFIG_CANSAT_GPS_UART_NUM;
    const uart_config_t config = {
        .baud_rate = CONFIG_CANSAT_GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t result = uart_driver_install(port, 2048, 0, 0, NULL, 0);
    if (result != ESP_OK) return result;
    s_driver_installed = true;
    result = uart_param_config(port, &config);
    if (result == ESP_OK) {
        result = uart_set_pin(port, CONFIG_CANSAT_GPS_TX_GPIO, CONFIG_CANSAT_GPS_RX_GPIO,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (result != ESP_OK) {
        (void)uart_driver_delete(port);
        s_driver_installed = false;
        return result;
    }
    s_ready = true;
    return ESP_OK;
}

esp_err_t gps_uart_recover(void)
{
    s_ready = false;
    if (s_driver_installed) {
        (void)uart_driver_delete((uart_port_t)CONFIG_CANSAT_GPS_UART_NUM);
        s_driver_installed = false;
    }
    return gps_uart_init();
}

int gps_uart_read(uint8_t *buffer, size_t capacity, uint32_t timeout_ms)
{
    if (buffer == NULL || capacity == 0U || !s_ready) return -1;
    return uart_read_bytes((uart_port_t)CONFIG_CANSAT_GPS_UART_NUM, buffer,
                           (uint32_t)capacity, pdMS_TO_TICKS(timeout_ms));
}
