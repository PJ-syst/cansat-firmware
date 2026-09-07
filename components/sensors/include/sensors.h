#ifndef CANSAT_SENSORS_H
#define CANSAT_SENSORS_H

#include "cansat_types.h"
#include "esp_err.h"

typedef struct {
    bool barometer_available;
    bool imu_available;
} sensors_health_t;

esp_err_t sensors_init(sensors_health_t *health);
esp_err_t sensors_recover(sensors_health_t *health);
esp_err_t sensors_read(sensor_reading_t *reading);
float sensors_altitude_from_pressure_kpa(float pressure_kpa);

#endif
