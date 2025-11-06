idf_component_register(
	SRCS "src/motor.c"
	INCLUDE_DIRS "include"
	REQUIRES "driver" 
        "esp_timer" 
        "esp_driver_gpio" 
        "esp_driver_i2c"
        "sensors"
        "i2c"
        "filter"
        "esp_driver_ledc"
)
