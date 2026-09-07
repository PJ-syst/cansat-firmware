#include "mechanism.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_CANSAT_MECHANISM_ENABLE
static bool s_armed;

static int safe_level(void)
{
    return CONFIG_CANSAT_MECHANISM_ACTIVE_HIGH ? 0 : 1;
}
#endif

esp_err_t mechanism_init(void)
{
#if CONFIG_CANSAT_MECHANISM_ENABLE
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << CONFIG_CANSAT_MECHANISM_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&config);
    if (result != ESP_OK) return result;
    s_armed = false;
    return gpio_set_level(CONFIG_CANSAT_MECHANISM_GPIO, safe_level());
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t mechanism_arm(void)
{
#if CONFIG_CANSAT_MECHANISM_ENABLE
    if (s_armed) return ESP_ERR_INVALID_STATE;
    s_armed = true;
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t mechanism_fire(void)
{
#if CONFIG_CANSAT_MECHANISM_ENABLE
    if (!s_armed) return ESP_ERR_INVALID_STATE;
    const int active = CONFIG_CANSAT_MECHANISM_ACTIVE_HIGH ? 1 : 0;
    esp_err_t result = gpio_set_level(CONFIG_CANSAT_MECHANISM_GPIO, active);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(CONFIG_CANSAT_MECHANISM_FIRE_MS));
    result = gpio_set_level(CONFIG_CANSAT_MECHANISM_GPIO, safe_level());
    s_armed = false;
    return result;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

void mechanism_force_safe(void)
{
#if CONFIG_CANSAT_MECHANISM_ENABLE
    (void)gpio_set_level(CONFIG_CANSAT_MECHANISM_GPIO, safe_level());
    s_armed = false;
#endif
}
