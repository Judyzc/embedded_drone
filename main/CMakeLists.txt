idf_component_register(
     SRCS "src/motors.c" 
          "src/controllers.c" 
          "src/estimator.c" 
          "src/sensors.c" 
          "src/main.c" 
          "src/six_axis_comp_filter.c" 
          "src/i2c_setup.c"
          "src/VL53L1X_api.c"
          "src/VL53L1X_calibration.c"
          "src/vl53l1_platform.c"
     INCLUDE_DIRS "." "inc"
     REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc
     )
