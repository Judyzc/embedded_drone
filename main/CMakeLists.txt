idf_component_register(SRCS "continuous_read_main.c"
                    PRIV_REQUIRES esp_adc esp_driver_dac esp_driver_gpio nvs_flash esp_event esp_wifi esp_system
                    INCLUDE_DIRS ".")
