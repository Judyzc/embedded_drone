idf_component_register(
     SRCS "VL53L1X_api.c"
          "VL53L1X_calibration.c"
          "vl53l1_platform.c"
     INCLUDE_DIRS "include"
     REQUIRES i2c_setup esp_driver_i2c
     )
