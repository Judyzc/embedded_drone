idf_component_register(
	SRCS "src/sensors.c"
	INCLUDE_DIRS "include"
	REQUIRES "driver"
		"i2c"
		"bosch"
		"freertos" 
		"driver" 
		"esp_driver_i2c" 
		"esp_driver_gpio" 
		"esp_timer" 
		"helper"
		"filter"
)
