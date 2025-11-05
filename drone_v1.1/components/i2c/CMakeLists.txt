idf_component_register(
	SRCS "src/i2c.c"
	INCLUDE_DIRS "include"
	REQUIRES "driver" 
        "esp_timer" 
        "esp_driver_gpio" 
        "esp_driver_i2c"
)
