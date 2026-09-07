idf_component_register(SRCS "sensors.c"
                       INCLUDE_DIRS "include"
                       REQUIRES cansat_types esp_driver_i2c esp_timer)
