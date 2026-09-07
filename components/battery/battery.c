#include "battery.h"

#include "sdkconfig.h"

#if CONFIG_CANSAT_BATTERY_ENABLE
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_calibration;
static bool s_calibrated;
#endif

esp_err_t battery_init(void)
{
#if CONFIG_CANSAT_BATTERY_ENABLE
    const adc_oneshot_unit_init_cfg_t unit_config = {.unit_id = ADC_UNIT_1};
    esp_err_t result = adc_oneshot_new_unit(&unit_config, &s_adc);
    if (result != ESP_OK) return result;
    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    result = adc_oneshot_config_channel(s_adc,
        (adc_channel_t)CONFIG_CANSAT_BATTERY_ADC_CHANNEL, &channel_config);
    if (result != ESP_OK) return result;
    const adc_cali_line_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100,
    };
    s_calibrated = adc_cali_create_scheme_line_fitting(&calibration_config,
                                                       &s_calibration) == ESP_OK;
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t battery_read(float *voltage_v, bool *voltage_valid,
                       float *current_a, bool *current_valid)
{
    if (voltage_v == NULL || voltage_valid == NULL ||
        current_a == NULL || current_valid == NULL) return ESP_ERR_INVALID_ARG;
    *voltage_valid = false;
    *current_valid = false;
    *voltage_v = 0.0f;
    *current_a = 0.0f;
#if CONFIG_CANSAT_BATTERY_ENABLE
    int total = 0;
    for (int i = 0; i < 8; ++i) {
        int raw = 0;
        esp_err_t result = adc_oneshot_read(s_adc,
            (adc_channel_t)CONFIG_CANSAT_BATTERY_ADC_CHANNEL, &raw);
        if (result != ESP_OK) return result;
        total += raw;
    }
    const int average = total / 8;
    int millivolts = 0;
    if (s_calibrated) {
        if (adc_cali_raw_to_voltage(s_calibration, average, &millivolts) != ESP_OK) {
            return ESP_FAIL;
        }
    } else {
        millivolts = (average * 3100) / 4095;
    }
    *voltage_v = ((float)millivolts / 1000.0f) *
                 ((float)CONFIG_CANSAT_BATTERY_DIVIDER_RATIO_MILLI / 1000.0f);
    *voltage_valid = true;
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
