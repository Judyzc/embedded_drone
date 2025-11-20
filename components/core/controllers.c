#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "pid_ctrl.h"
#include "math.h"

#include "controllers.h"
// #include "sensors.h"
// #include "six_axis_comp_filter.h"
#include "estimator.h"
#include "motors.h"

static const char *TAG = "controllers";
/* ------------------------------------------- Private Global Variables ------------------------------------------- */
static pid_ctrl_block_handle_t pitch_pid_handle, pitch_rate_pid_handle, roll_pid_handle, roll_rate_pid_handle, 
                               altitude_pid_handle, altitude_rate_pid_handle, vel_x_pid_handle, vel_y_pid_handle,
                               yaw_rate_pid_handle;
static QueueHandle_t xQueue_state_data; 
static bool EMERG_STOP = false; 
static int fault_count = 0; 

/* ------------------------------------------- Private Function Definitions ------------------------------------------- */
static float limit_motor_cmd(float motor_cmd) {
    if (motor_cmd > MAX_DUTY_CYCLE_PCT) {
        return MAX_DUTY_CYCLE_PCT; 
    } else if (motor_cmd<0.0) {
        return 0.0; 
    } else {
        return motor_cmd; 
    }
}

static motor_cmds_t sum_motor_cmds(float pitch_cmd, float roll_cmd, float thrust_cmd, float yaw_cmd) {
    // pitch and roll
    motor_cmds_t motor_cmds = {
        .motor1_duty_cycle_pct = limit_motor_cmd(pitch_cmd + roll_cmd + thrust_cmd + yaw_cmd),
        .motor2_duty_cycle_pct = limit_motor_cmd(pitch_cmd - roll_cmd + thrust_cmd - yaw_cmd),
        .motor3_duty_cycle_pct = limit_motor_cmd(-1.0*pitch_cmd - roll_cmd + thrust_cmd + yaw_cmd),
        .motor4_duty_cycle_pct = limit_motor_cmd(-1.0*pitch_cmd + roll_cmd + thrust_cmd - yaw_cmd),
    };

    return motor_cmds; 
}

void vUpdatePIDTask(void *pvParameters) {
    state_data_t state_data; 
    for (;;) {
        // Check for unstable flight -> kill motors
        if (!EMERG_STOP) {
            if (fabs(state_data.pitch_rad) > M_PI/4.0 || fabs(state_data.roll_rad) > M_PI/4.0) {
                fault_count++; 
                if (fault_count > 10) {
                    ESP_LOGE(TAG, "Stopping Motors"); 
                    EMERG_STOP = true; 
                }
            } else {
                fault_count = 0; 
            }
        }

        /* ----------------------------- Pitch cascaded PIDs ----------------------------- */
        xQueueReceive(xQueue_state_data, (void *) &state_data, portMAX_DELAY); 

        float vel_y_error_m_s = 0.0 - state_data.vel_y_m_s; 
        float desired_pitch_rad; 
        pid_compute(vel_y_pid_handle, vel_y_error_m_s, &desired_pitch_rad);
        desired_pitch_rad *= -1.0; 
        desired_pitch_rad = 0;                  // For tuning the second PID

        float pitch_error_rad = desired_pitch_rad - state_data.pitch_rad; 
        float desired_pitch_rate_rad_s; 
        pid_compute(pitch_pid_handle, pitch_error_rad, &desired_pitch_rate_rad_s);
        // desired_pitch_rate_rad_s = 0;           // For tuning the third PID

        float pitch_rate_error = desired_pitch_rate_rad_s - state_data.pitch_rate_rad_s; 
        float pitch_cmd; 
        pid_compute(pitch_rate_pid_handle, pitch_rate_error, &pitch_cmd); 
        // pitch_cmd = 0;                          // For tuning other PIDs
        
        /* ----------------------------- Roll cascaded PIDs ----------------------------- */
        float vel_x_error_m_s = 0.0 - state_data.vel_x_m_s; 
        float desired_roll_rad; 
        pid_compute(vel_x_pid_handle, vel_x_error_m_s, &desired_roll_rad);
        desired_roll_rad = 0;               // For tuning the second PID

        float roll_error_rad = desired_roll_rad - state_data.roll_rad; 
        float desired_roll_rate_rad_s; 
        pid_compute(roll_pid_handle, roll_error_rad, &desired_roll_rate_rad_s);
        // desired_roll_rate_rad_s = 0;        // For tuning the third PID
     
        float roll_rate_error = desired_roll_rate_rad_s - state_data.roll_rate_rad_s; 
        float roll_cmd; 
        pid_compute(roll_rate_pid_handle, roll_rate_error, &roll_cmd);
        // roll_cmd = 0;                           // For tuning other PIDs

        /* ----------------------------- Yaw rate PID ----------------------------- */
        float desired_yaw_rate_rad_s = 0; 
        float yaw_rate_error_rad_s = desired_yaw_rate_rad_s - state_data.yaw_rate_rad_s; 
        float yaw_cmd; 
        pid_compute(yaw_rate_pid_handle, yaw_rate_error_rad_s, &yaw_cmd);
        // yaw_cmd = 0;        // For tuning the other PIDs

        /* ----------------------------- Altitude cascaded PIDs ----------------------------- */
        float altitdue_error_m = .5 - state_data.altitude_m; 
        float desired_altitude_rate_m_s; 
        pid_compute(altitude_pid_handle, altitdue_error_m, &desired_altitude_rate_m_s);
        // desired_altitude_rate_m_s = 0;        // For tuning second PID

        float altitude_rate_error = desired_altitude_rate_m_s - state_data.altitude_rate_m_s; 
        float thrust_cmd; 
        pid_compute(altitude_rate_pid_handle, altitude_rate_error, &thrust_cmd);
        // thrust_cmd = 0;                       // For tuning pitch and roll PIDs

        /* ----------------------------- Sum commands and send to motors ----------------------------- */
        motor_cmds_t motor_cmds = sum_motor_cmds(pitch_cmd, roll_cmd, thrust_cmd, yaw_cmd); 

        if (EMERG_STOP) {
            motor_cmds.motor1_duty_cycle_pct = 0; 
            motor_cmds.motor2_duty_cycle_pct = 0; 
            motor_cmds.motor3_duty_cycle_pct = 0; 
            motor_cmds.motor4_duty_cycle_pct = 0;  
        }

        update_pwm(motor_cmds); 
    }
}

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(QueueHandle_t *pxQueue_state_data) {
    // Save Queue
    xQueue_state_data = *pxQueue_state_data; 

    // Initialize each PID controller 
    pid_ctrl_parameter_t vel_y_pid_runtime_param = {
        .kp = VEL_KP,
        .ki = VEL_KI*DT,
        .kd = VEL_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = VEL_LIMIT,
        .min_output   = -1.0*VEL_LIMIT,
        .max_integral = VEL_LIMIT,
        .min_integral = -1.0*VEL_LIMIT,
    };
    pid_ctrl_config_t vel_y_pid_config = {
        .init_param = vel_y_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&vel_y_pid_config, &vel_y_pid_handle));

    pid_ctrl_parameter_t pitch_pid_runtime_param = {
        .kp = ATTITUDE_KP,
        .ki = ATTITUDE_KI*DT,
        .kd = ATTITUDE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = ATTITUDE_LIMIT,
        .min_output   = -1.0*ATTITUDE_LIMIT,
        .max_integral = ATTITUDE_LIMIT,
        .min_integral = -1.0*ATTITUDE_LIMIT,
    };
    pid_ctrl_config_t pitch_pid_config = {
        .init_param = pitch_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pitch_pid_config, &pitch_pid_handle));

    pid_ctrl_parameter_t pitch_rate_pid_runtime_param = {
        .kp = ATTITUDE_RATE_KP,
        .ki = ATTITUDE_RATE_KI*DT,
        .kd = ATTITUDE_RATE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = ATTITUDE_RATE_LIMIT,
        .min_output   = -1.0*ATTITUDE_RATE_LIMIT,
        .max_integral = ATTITUDE_RATE_LIMIT,
        .min_integral = -1.0*ATTITUDE_RATE_LIMIT,
    };
    pid_ctrl_config_t pitch_rate_pid_config = {
        .init_param = pitch_rate_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pitch_rate_pid_config, &pitch_rate_pid_handle));
    
    pid_ctrl_parameter_t vel_x_pid_runtime_param = {
        .kp = VEL_KP,
        .ki = VEL_KI*DT,
        .kd = VEL_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = VEL_LIMIT,
        .min_output   = -1.0*VEL_LIMIT,
        .max_integral = VEL_LIMIT,
        .min_integral = -1.0*VEL_LIMIT,
    };
    pid_ctrl_config_t vel_x_pid_config = {
        .init_param = vel_x_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&vel_x_pid_config, &vel_x_pid_handle));

    pid_ctrl_parameter_t roll_pid_runtime_param = {
        .kp = ATTITUDE_KP,
        .ki = ATTITUDE_KI*DT,
        .kd = ATTITUDE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = ATTITUDE_LIMIT,
        .min_output   = -1.0*ATTITUDE_LIMIT,
        .max_integral = ATTITUDE_LIMIT,
        .min_integral = -1.0*ATTITUDE_LIMIT,
    };
    pid_ctrl_config_t roll_pid_config = {
        .init_param = roll_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&roll_pid_config, &roll_pid_handle));

    pid_ctrl_parameter_t roll_rate_pid_runtime_param = {
        .kp = ATTITUDE_RATE_KP,
        .ki = ATTITUDE_RATE_KI*DT,
        .kd = ATTITUDE_RATE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = ATTITUDE_RATE_LIMIT,
        .min_output   = -1.0*ATTITUDE_RATE_LIMIT,
        .max_integral = ATTITUDE_RATE_LIMIT,
        .min_integral = -1.0*ATTITUDE_RATE_LIMIT,
    };
    pid_ctrl_config_t roll_rate_pid_config = {
        .init_param = roll_rate_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&roll_rate_pid_config, &roll_rate_pid_handle));

    pid_ctrl_parameter_t yaw_rate_pid_runtime_param = {
        .kp = YAW_RATE_KP,
        .ki = YAW_RATE_KI*DT,
        .kd = YAW_RATE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = YAW_RATE_LIMIT,
        .min_output   = -1.0*YAW_RATE_LIMIT,
        .max_integral = YAW_RATE_LIMIT,
        .min_integral = -1.0*YAW_RATE_LIMIT,
    };
    pid_ctrl_config_t yaw_rate_pid_config = {
        .init_param = yaw_rate_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&yaw_rate_pid_config, &yaw_rate_pid_handle));

    pid_ctrl_parameter_t altitude_pid_runtime_param = {
        .kp = ALTITUDE_KP,
        .ki = ALTITUDE_KI*DT,
        .kd = ALTITUDE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = ALTITUDE_LIMIT,
        .min_output   = -1.0*ALTITUDE_LIMIT,
        .max_integral = ALTITUDE_LIMIT,
        .min_integral = -1.0*ALTITUDE_LIMIT,
    };
    pid_ctrl_config_t altitude_pid_config = {
        .init_param = altitude_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&altitude_pid_config, &altitude_pid_handle));

    pid_ctrl_parameter_t altitude_rate_pid_runtime_param = {
        .kp = ALTITUDE_RATE_KP,
        .ki = ALTITUDE_RATE_KI*DT,
        .kd = ALTITUDE_RATE_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = ALTITUDE_RATE_LIMIT,
        .min_output   = -1.0*ALTITUDE_RATE_LIMIT,
        .max_integral = ALTITUDE_RATE_LIMIT,
        .min_integral = -1.0*ALTITUDE_RATE_LIMIT,
    };
    pid_ctrl_config_t altitude_rate_pid_config = {
        .init_param = altitude_rate_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&altitude_rate_pid_config, &altitude_rate_pid_handle));

    ESP_LOGI(TAG, "Initialized PIDs successfully");

    // Start PID update task
    xTaskCreate(vUpdatePIDTask, "Cascaded PID", 4096, NULL, ESTIMATOR_PRIORITY, NULL);
}