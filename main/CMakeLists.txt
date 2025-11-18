idf_component_register(
     SRCS "main.c"
     INCLUDE_DIRS "."
     REQUIRES esp_driver_gpio 
              esp_driver_i2c 
              esp_driver_ledc
              core 
              bmi088
              digital_io
              espnow
              i2c_setup
              pmw3901
              spi_setup
              vl53l1x
     )
