#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "pid_ctrl.h"
#include "math.h"

#include "main.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"
#include "controllers.h"
#include "motors.h"
#include "driver/gpio.h"

static const char *TAG = "controllers";
/* ------------------------------------------- Global Variables ------------------------------------------- */
pid_ctrl_block_handle_t pitch_pid_handle, pitch_rate_pid_handle, roll_pid_handle, roll_rate_pid_handle;

bool EMERG_STOP = false; 

/* ------------------------------------------- Private Function Definitions ------------------------------------------- */
// Placeholder
static float torque_2_force(float torque_Nm) {
    return torque_Nm/MOTOR_MOMENT_ARM_M;  
}

// Placeholder
static float force_2_duty_cycle(float force_N) {
    float duty_cycle = force_N/MAX_THRUST_N*100.0; 
    if (duty_cycle > MAX_DUTY_CYCLE_PCT) {
        duty_cycle = MAX_DUTY_CYCLE_PCT; 
    } else if (duty_cycle < -1.0*MAX_DUTY_CYCLE_PCT) {
        duty_cycle = -1.0*MAX_DUTY_CYCLE_PCT; 
    }
    return duty_cycle; 
}

static motor_cmds_t sum_motor_cmds(float pitch_torque_cmd_Nm, float roll_torque_cmd_Nm) {
    float pitch_force_cmd_N = torque_2_force(pitch_torque_cmd_Nm); 
    float roll_force_cmd_N = torque_2_force(roll_torque_cmd_Nm); 

    // pitch and roll
    motor_cmds_t motor_cmds = {
        .motor1_duty_cycle_pct = force_2_duty_cycle(.25*(pitch_force_cmd_N + roll_force_cmd_N)),
        .motor2_duty_cycle_pct = force_2_duty_cycle(.25*(pitch_force_cmd_N - roll_force_cmd_N)),
        .motor3_duty_cycle_pct = force_2_duty_cycle(.25*(-1.0*pitch_force_cmd_N - roll_force_cmd_N)),
        .motor4_duty_cycle_pct = force_2_duty_cycle(.25*(-1.0*pitch_force_cmd_N + roll_force_cmd_N)),
    };

    return motor_cmds; 
}

void vUpdatePIDTask(void *pvParameters) {
    state_data_t state_data; 
    for (;;) {
        EMERG_STOP = true; 
        // Check for unstable flight -> kill motors
        if (!EMERG_STOP && (fabs(state_data.pitch_rad) > M_PI/4.0 || fabs(state_data.roll_rad) > M_PI/4.0)) {
            ESP_LOGE(TAG, "Stopping Motors"); 
            EMERG_STOP = true; 
        }


        // Pitch cascaded PIDs
        xQueueReceive(xQueue_state_data, (void *) &state_data, portMAX_DELAY); 

        float pitch_error_rad = 0.0 - state_data.pitch_rad; 
        float desired_pitch_rate_rad_s; 
        pid_compute(pitch_pid_handle, pitch_error_rad, &desired_pitch_rate_rad_s);
        // desired_pitch_rate_rad_s = 0;       // For tuning the second PID

        float pitch_rate_error = desired_pitch_rate_rad_s - state_data.pitch_rate_rad_s; 
        float pitch_torque_cmd_Nm; 
        pid_compute(pitch_rate_pid_handle, pitch_rate_error, &pitch_torque_cmd_Nm); 
        
        // Roll cascaded PIDs 
        float roll_error_rad = 0.0 - state_data.roll_rad; 
        float desired_roll_rate_rad_s; 
        pid_compute(roll_pid_handle, roll_error_rad, &desired_roll_rate_rad_s);

        // desired_roll_rate_rad_s = 0;        // For tuning second PID
     
        float roll_rate_error = desired_roll_rate_rad_s - state_data.roll_rate_rad_s; 
        float roll_torque_cmd_Nm; 
        pid_compute(roll_rate_pid_handle, roll_rate_error, &roll_torque_cmd_Nm); 

        motor_cmds_t motor_cmds = sum_motor_cmds(pitch_torque_cmd_Nm, roll_torque_cmd_Nm); 

        if (EMERG_STOP) {
            motor_cmds.motor1_duty_cycle_pct = 0; 
            motor_cmds.motor2_duty_cycle_pct = 0; 
            motor_cmds.motor3_duty_cycle_pct = 0; 
            motor_cmds.motor4_duty_cycle_pct = 0;  
        }

        update_pwm(motor_cmds);
        end_tick = xTaskGetTickCount(); 

        // ESP_LOGI(TAG, "Sensor to Motor Time: %d ticks", end_tick - start_tick);  
    }
}

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(void) {
    // Initialize each PID controller 
    pid_ctrl_parameter_t pitch_pid_runtime_param = {
        .kp = PITCH_KP,
        .ki = PITCH_KI*DT,
        .kd = PITCH_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = PITCH_LIMIT,
        .min_output   = -1.0*PITCH_LIMIT,
        .max_integral = PITCH_LIMIT,
        .min_integral = -1.0*PITCH_LIMIT,
    };
    pid_ctrl_config_t pitch_pid_config = {
        .init_param = pitch_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pitch_pid_config, &pitch_pid_handle));

    pid_ctrl_parameter_t pitch_rate_pid_runtime_param = {
        .kp = PITCH_RATE_KP,
        .ki = PITCH_RATE_KI*DT,
        .kd = PITCH_RATE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = PITCH_RATE_LIMIT,
        .min_output   = -1.0*PITCH_RATE_LIMIT,
        .max_integral = PITCH_RATE_LIMIT,
        .min_integral = -1.0*PITCH_RATE_LIMIT,
    };
    pid_ctrl_config_t pitch_rate_pid_config = {
        .init_param = pitch_rate_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pitch_rate_pid_config, &pitch_rate_pid_handle));
    
    pid_ctrl_parameter_t roll_pid_runtime_param = {
        .kp = ROLL_KP,
        .ki = ROLL_KI*DT,
        .kd = ROLL_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = ROLL_LIMIT,
        .min_output   = -1.0*ROLL_LIMIT,
        .max_integral = ROLL_LIMIT,
        .min_integral = -1.0*ROLL_LIMIT,
    };
    pid_ctrl_config_t roll_pid_config = {
        .init_param = roll_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&roll_pid_config, &roll_pid_handle));

    pid_ctrl_parameter_t roll_rate_pid_runtime_param = {
        .kp = ROLL_RATE_KP,
        .ki = ROLL_RATE_KI*DT,
        .kd = ROLL_RATE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = ROLL_RATE_LIMIT,
        .min_output   = -1.0*ROLL_RATE_LIMIT,
        .max_integral = ROLL_RATE_LIMIT,
        .min_integral = -1.0*ROLL_RATE_LIMIT,
    };
    pid_ctrl_config_t roll_rate_pid_config = {
        .init_param = roll_rate_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&roll_rate_pid_config, &roll_rate_pid_handle));

    ESP_LOGI(TAG, "Initialized PIDs successfully");

    // Start PID update task
    xTaskCreate(vUpdatePIDTask, "Cascaded PID", 4096, NULL, ESTIMATOR_PRIORITY, NULL);
}