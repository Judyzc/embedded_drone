idf_component_register(
			SRCS "src/bmp3.c"
                "src/bstdr_comm_support.c"
			INCLUDE_DIRS "interface"
			REQUIRES "driver" "esp_timer" "esp_driver_gpio"
)
