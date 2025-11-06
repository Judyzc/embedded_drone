idf_component_register(
	SRCS "src/complementary_filter.c"
		"src/six_axis_comp_filter.c"
	INCLUDE_DIRS "include"
	REQUIRES "driver"
		"freertos" 
		"sensors"
		
)
