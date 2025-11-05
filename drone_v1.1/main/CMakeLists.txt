idf_component_register(
    SRCS    "main.c"
    INCLUDE_DIRS "."
    REQUIRES "freertos" 
            "driver" 
            "esp_driver_i2c" 
            "esp_driver_gpio" 
            "esp_timer" 
            # components
            "bosch"
            "sensors"

            "filter"
            "controller"
            
            "i2c"
            "helper"
)