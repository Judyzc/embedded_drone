idf_component_register(
    SRCS    "main.c"
            "complementary_filter.h"  
            "sensors.h"           
    INCLUDE_DIRS "."
    REQUIRES vl53l0x freertos driver
)