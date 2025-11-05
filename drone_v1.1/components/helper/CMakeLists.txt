idf_component_register(
	SRCS "src/helper.c"
	INCLUDE_DIRS "include"
	REQUIRES "driver"
		"sensors"
		"i2c"
		"freertos" 
		"driver" 
		"esp_driver_i2c" 
		"esp_driver_gpio" 
		"esp_timer" 
		
)
