idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES cansat_types command flight_state gps telemetry
                                sensors radio battery mechanism esp_timer esp_system)
