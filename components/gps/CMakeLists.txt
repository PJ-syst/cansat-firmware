idf_component_register(SRCS "nmea_parser.c" "gps_uart.c"
                       INCLUDE_DIRS "include"
                       REQUIRES cansat_types esp_driver_uart)
