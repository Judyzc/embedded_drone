#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "pid_ctrl.h"

#include "main.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"
#include "controllers.h"

static const char *TAG = "controllers";
/* ------------------------------------------- Global Variables ------------------------------------------- */
pid_ctrl_block_handle_t pitch_pid_handle;
pid_ctrl_block_handle_t pitch_pid_rate_handle;

/* ------------------------------------------- Private Function Definitions ------------------------------------------- */
// Placeholder
static float torque_2_force(float torque_Nm) {
    return torque_Nm/1.0; 
}

// Placeholder
static float force_2_duty_cycle(float force_N) {
    return force_N*100.0; 
}

void vUpdatePIDTask(void *pvParameters) {
    state_data_t state_data; 
    for (;;) {
        xQueueReceive(xQueue_state_data, (void *) &state_data, portMAX_DELAY); 
        float pitch_error_rad = 0.0 - state_data.pitch_rad; 
        float desired_pitch_rate_rad_s; 
        pid_compute(pitch_pid_handle, pitch_error_rad, &desired_pitch_rate_rad_s);
        
        float pitch_rate_error = desired_pitch_rate_rad_s - state_data.pitch_rate_rad_s; 
        float pitch_torque_cmd_N_m; 
        pid_compute(pitch_pid_rate_handle, pitch_rate_error, &pitch_torque_cmd_N_m); 

        float pitch_force_cmd_N = torque_2_force(pitch_torque_cmd_N_m); 
        float pitch_cmd_duty_cycle_pct = force_2_duty_cycle(pitch_force_cmd_N); 
    }
}

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(void) {
    // Initialize each PID controller 
    pid_ctrl_parameter_t pitch_pid_runtime_param = {
        .kp = 1.0,
        .ki = 1.0,
        .kd = 1.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = 1.0,
        .min_output   = -1.0,
        .max_integral = 1.0,
        .min_integral = -1.0,
    };
    pid_ctrl_config_t pitch_pid_config = {
        .init_param = pitch_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pitch_pid_config, &pitch_pid_handle));

    pid_ctrl_parameter_t pitch_pid_rate_runtime_param = {
        .kp = 1.0,
        .ki = 1.0,
        .kd = 1.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = 1.0,
        .min_output   = -1.0,
        .max_integral = 1.0,
        .min_integral = -1.0,
    };
    pid_ctrl_config_t pitch_pid_rate_config = {
        .init_param = pitch_pid_rate_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pitch_pid_config, &pitch_pid_rate_handle));

    // Start PID update task
    xTaskCreate(vUpdatePIDTask, "Cascaded PID", 4096, NULL, ESTIMATOR_PRIORITY, NULL);
}