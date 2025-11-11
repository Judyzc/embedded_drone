idf_component_register(SRCS 
  "core/VL53L1X_api.c"
  "core/VL53L1X_calibration.c"
  "platform/VL53L1_platform.c"
  INCLUDE_DIRS
    "core"
    "platform"
  REQUIRES mbedtls log driver esp_timer freertos)

