idf_component_register(SRCS "motors.c" "controllers.c" "estimator.c" "sensors.c" "main.c" "six_axis_comp_filter.c"
                       INCLUDE_DIRS "."
                       REQUIRES VL53L1_ULD_ESP)
