idf_component_register(
	SRCS "src/controller.c"
	INCLUDE_DIRS "include"
	REQUIRES "driver"
		"esp_driver_ledc"
        "filter"
		"filter"
		"sensors"
		"pid_ctrl"
)
