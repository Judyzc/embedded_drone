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
                               altitude_pid_handle, altitude_rate_pid_handle, vel_x_pid_handle, vel_y_pid_handle;
static QueueHandle_t xQueue_state_data; 
static bool EMERG_STOP = false; 

/* ------------------------------------------- Private Function Definitions ------------------------------------------- */
static float torque_2_force(float torque_Nm) {
    return torque_Nm/MOTOR_MOMENT_ARM_M;  
}

static float force_2_duty_cycle(float force_N) {
    float duty_cycle = force_N/MAX_THRUST_N*100.0; 
    if (duty_cycle > MAX_DUTY_CYCLE_PCT) {
        // ESP_LOGI(TAG, "Hitting max duty cycle"); 
        duty_cycle = MAX_DUTY_CYCLE_PCT; 
    } else if (duty_cycle < -1.0*MAX_DUTY_CYCLE_PCT) {
        duty_cycle = -1.0*MAX_DUTY_CYCLE_PCT; 
        // ESP_LOGI(TAG, "Hitting max duty cycle"); 
    }
    return duty_cycle; 
}

static motor_cmds_t sum_motor_cmds(float pitch_torque_cmd_Nm, float roll_torque_cmd_Nm, float thrust_cmd_N) {
    float pitch_force_cmd_N = torque_2_force(pitch_torque_cmd_Nm); 
    float roll_force_cmd_N = torque_2_force(roll_torque_cmd_Nm); 

    // pitch and roll
    motor_cmds_t motor_cmds = {
        .motor1_duty_cycle_pct = force_2_duty_cycle(.25*(pitch_force_cmd_N + roll_force_cmd_N + thrust_cmd_N)),
        .motor2_duty_cycle_pct = force_2_duty_cycle(.25*(pitch_force_cmd_N - roll_force_cmd_N + thrust_cmd_N)),
        .motor3_duty_cycle_pct = force_2_duty_cycle(.25*(-1.0*pitch_force_cmd_N - roll_force_cmd_N + thrust_cmd_N)),
        .motor4_duty_cycle_pct = force_2_duty_cycle(.25*(-1.0*pitch_force_cmd_N + roll_force_cmd_N + thrust_cmd_N)),
    };

    return motor_cmds; 
}

void vUpdatePIDTask(void *pvParameters) {
    state_data_t state_data; 
    for (;;) {
        // Check for unstable flight -> kill motors
        if (!EMERG_STOP && (fabs(state_data.pitch_rad) > M_PI/4.0 || fabs(state_data.roll_rad) > M_PI/4.0)) {
            ESP_LOGE(TAG, "Stopping Motors"); 
            EMERG_STOP = true; 
        }

        gpio_set_level(CONFIG_PIN_TOGGLE_B, 1);

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
        float pitch_torque_cmd_Nm; 
        pid_compute(pitch_rate_pid_handle, pitch_rate_error, &pitch_torque_cmd_Nm); 
        
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
        float roll_torque_cmd_Nm; 
        pid_compute(roll_rate_pid_handle, roll_rate_error, &roll_torque_cmd_Nm);

        /* ----------------------------- Altitude cascaded PIDs ----------------------------- */
        float altitdue_error_m = .5 - state_data.altitude_m; 
        float desired_altitude_rate_m_s; 
        pid_compute(altitude_pid_handle, altitdue_error_m, &desired_altitude_rate_m_s);
        // desired_roll_rate_rad_s = 0;        // For tuning second PID

        float altitude_rate_error = desired_altitude_rate_m_s - state_data.altitude_rate_m_s; 
        float thrust_cmd_N; 
        pid_compute(altitude_rate_pid_handle, altitude_rate_error, &thrust_cmd_N);
        // thrust_cmd_N = 0;                       // For tuning pitch and roll PIDs

        motor_cmds_t motor_cmds = sum_motor_cmds(pitch_torque_cmd_Nm, roll_torque_cmd_Nm, thrust_cmd_N); 

        if (EMERG_STOP) {
            motor_cmds.motor1_duty_cycle_pct = 0; 
            motor_cmds.motor2_duty_cycle_pct = 0; 
            motor_cmds.motor3_duty_cycle_pct = 0; 
            motor_cmds.motor4_duty_cycle_pct = 0;  
        }

        update_pwm(motor_cmds);

        gpio_set_level(CONFIG_PIN_TOGGLE_B, 0);
        // ESP_LOGI(TAG, "Sensor to Motor Time: %d ticks", end_tick - start_tick);  
    }
}

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(QueueHandle_t *pxQueue_state_data) {
    // Save Queue
    xQueue_state_data = *pxQueue_state_data; 

    // Initialize each PID controller 
    pid_ctrl_parameter_t vel_x_pid_runtime_param = {
        .kp = VEL_X_KP,
        .ki = VEL_X_KI*DT,
        .kd = VEL_X_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = VEL_X_LIMIT,
        .min_output   = -1.0*VEL_X_LIMIT,
        .max_integral = VEL_X_LIMIT,
        .min_integral = -1.0*VEL_X_LIMIT,
    };
    pid_ctrl_config_t vel_x_pid_config = {
        .init_param = vel_x_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&vel_x_pid_config, &vel_x_pid_handle));

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
    
    pid_ctrl_parameter_t vel_y_pid_runtime_param = {
        .kp = VEL_Y_KP,
        .ki = VEL_Y_KI*DT,
        .kd = VEL_Y_KD/DT,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = VEL_Y_LIMIT,
        .min_output   = -1.0*VEL_Y_LIMIT,
        .max_integral = VEL_Y_LIMIT,
        .min_integral = -1.0*VEL_Y_LIMIT,
    };
    pid_ctrl_config_t vel_y_pid_config = {
        .init_param = vel_y_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&vel_y_pid_config, &vel_y_pid_handle));

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