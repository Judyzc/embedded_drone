#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "controller.h"
#include "math.h"

#include "pid_ctrl.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "complementary_filter.h"
#include "controller.h"
#include "motor.h"

static const char *TAG = "controllers";
/* ------------------------------------------- Global Variables ------------------------------------------- */
pid_ctrl_block_handle_t pitch_pid_handle, pitch_rate_pid_handle, roll_pid_handle, roll_rate_pid_handle;

TickType_t start_tick, end_tick;

bool EMERG_STOP = false; 

/* ------------------------------------------- Private Function Definitions ------------------------------------------- */
// Placeholder
static float torque_2_force(float torque_Nm) {
    return torque_Nm/1.0; 
}

// Placeholder
static float force_2_duty_cycle(float force_N) {
    float duty_cycle = force_N*100.0; 
    if (duty_cycle > 25.0) {
        duty_cycle = 25.0; 
    } else if (duty_cycle < -25.0) {
        duty_cycle = -25.0; 
    }
    return duty_cycle; 
}

static motor_cmds_t sum_motor_cmds(float pitch_torque_cmd, float roll_torque_cmd) {
    float pitch_force_cmd = torque_2_force(pitch_torque_cmd); // N
    float roll_force_cmd = torque_2_force(roll_torque_cmd); // N

    // pitch and roll
    motor_cmds_t motor_cmds = {
        .motor1_duty_cycle_pct = force_2_duty_cycle(pitch_force_cmd + roll_force_cmd),
        .motor2_duty_cycle_pct = force_2_duty_cycle(pitch_force_cmd - roll_force_cmd),
        .motor3_duty_cycle_pct = force_2_duty_cycle(-1.0*pitch_force_cmd - roll_force_cmd),
        .motor4_duty_cycle_pct = force_2_duty_cycle(-1.0*pitch_force_cmd + roll_force_cmd),
    };

    // just pitch
    // motor_cmds_t motor_cmds = {
    //     .motor1_duty_cycle_pct = force_2_duty_cycle(pitch_force_cmd),
    //     .motor2_duty_cycle_pct = force_2_duty_cycle(pitch_force_cmd),
    //     .motor3_duty_cycle_pct = force_2_duty_cycle(-1.0*pitch_force_cmd),
    //     .motor4_duty_cycle_pct = force_2_duty_cycle(-1.0*pitch_force_cmd),
    // }; 

    // just roll
    // motor_cmds_t motor_cmds = {
    //     .motor1_duty_cycle_pct = force_2_duty_cycle(roll_force_cmd),
    //     .motor2_duty_cycle_pct = force_2_duty_cycle(-1.0*roll_force_cmd),
    //     .motor3_duty_cycle_pct = force_2_duty_cycle(-1.0*roll_force_cmd),
    //     .motor4_duty_cycle_pct = force_2_duty_cycle(roll_force_cmd),
    // };

    return motor_cmds; 
}

void controller_update(estimated_state_t *estimated_state) {
    start_tick = xTaskGetTickCount(); 
    // Check for unstable flight -> kill motors
    float attitude_bounds = M_PI/4.0;
    if (estimated_state->pitch > attitude_bounds || estimated_state->pitch < -attitude_bounds || estimated_state->roll > attitude_bounds || estimated_state->roll < -attitude_bounds) {
        ESP_LOGW(TAG, "Killing motors, attitude out of bounds."); 
        EMERG_STOP = true; 
    }

    // Pitch cascaded PIDs
    float pitch_error = 0.0 - estimated_state->pitch; 
    float desired_pitch_rate; // rad/s
    pid_compute(pitch_pid_handle, pitch_error, &desired_pitch_rate);
    
    float pitch_rate_error = desired_pitch_rate - estimated_state->pitch_rate; 
    float pitch_torque_cmd; // Nm
    pid_compute(pitch_rate_pid_handle, pitch_rate_error, &pitch_torque_cmd); 
    
    // Roll cascaded PIDs 
    float roll_error = 0.0 - estimated_state->roll; 
    float desired_roll_rate; // rad/s
    pid_compute(roll_pid_handle, roll_error, &desired_roll_rate);
    
    float roll_rate_error = desired_roll_rate - estimated_state->roll_rate; 
    float roll_torque_cmd; // Nm
    pid_compute(roll_rate_pid_handle, roll_rate_error, &roll_torque_cmd); 

    motor_cmds_t motor_cmds = sum_motor_cmds(pitch_torque_cmd, roll_torque_cmd); 
    if (EMERG_STOP) {
        motor_cmds.motor1_duty_cycle_pct = 0; 
        motor_cmds.motor2_duty_cycle_pct = 0; 
        motor_cmds.motor3_duty_cycle_pct = 0; 
        motor_cmds.motor4_duty_cycle_pct = 0;  
    }

    update_pwm(motor_cmds);
    end_tick = xTaskGetTickCount(); 
    
    ESP_LOGI(TAG, "Sensor to Motor Time: %d ticks", end_tick - start_tick);  
}

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controller_init(void) {
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

    pid_ctrl_parameter_t pitch_rate_pid_runtime_param = {
        .kp = 1.0,
        .ki = 1.0,
        .kd = 1.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = 1.0,
        .min_output   = -1.0,
        .max_integral = 1.0,
        .min_integral = -1.0,
    };
    pid_ctrl_config_t pitch_rate_pid_config = {
        .init_param = pitch_rate_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pitch_rate_pid_config, &pitch_rate_pid_handle));
    
    pid_ctrl_parameter_t roll_pid_runtime_param = {
        .kp = 1.0,
        .ki = 1.0,
        .kd = 1.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = 1.0,
        .min_output   = -1.0,
        .max_integral = 1.0,
        .min_integral = -1.0,
    };
    pid_ctrl_config_t roll_pid_config = {
        .init_param = roll_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&roll_pid_config, &roll_pid_handle));

    pid_ctrl_parameter_t roll_rate_pid_runtime_param = {
        .kp = 1.0,
        .ki = 1.0,
        .kd = 1.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = 1.0,
        .min_output   = -1.0,
        .max_integral = 1.0,
        .min_integral = -1.0,
    };
    pid_ctrl_config_t roll_rate_pid_config = {
        .init_param = roll_rate_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&roll_rate_pid_config, &roll_rate_pid_handle));

    // // Start PID update task
    // xTaskCreate(vUpdatePIDTask, "Cascaded PID", 4096, NULL, ESTIMATOR_PRIORITY, NULL);
}