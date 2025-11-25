idf_component_register(
     SRCS "motors.c" 
          "controllers.c" 
          "estimator.c" 
          "sensors.c"
          "six_axis_comp_filter.c" 
          "esp_task.c" 
     INCLUDE_DIRS "include"
     REQUIRES esp_driver_gpio 
              esp_driver_i2c 
              esp_driver_ledc 
              espressif__pid_ctrl 
              i2c_setup 
              spi_setup 
              bmi088
              vl53l1x 
              pmw3901 
              nvs_flash 
              esp_event 
              esp_wifi 
              esp_system
     )
