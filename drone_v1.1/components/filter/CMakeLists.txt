idf_component_register(
	SRCS "src/complementary_filter.c"
	INCLUDE_DIRS "include"
	REQUIRES "driver"
		"freertos" 
		# "esp_driver_i2c" 
		# "esp_driver_gpio" 
		# "esp_timer" 
		"sensors"
		
)
