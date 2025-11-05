#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "math.h"

#include "main.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"
#include "motors.h"

static const char *TAG = "motors";

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void motors_init(void) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RES,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQUENCY_HZ,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    // Prepare and then apply the LEDC PWM channel configurations for each motor
    ledc_channel_config_t motor_1_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = MOTOR_1_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_1_GPIO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_1_channel));

    ledc_channel_config_t motor_2_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = MOTOR_2_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_2_GPIO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_2_channel));

    ledc_channel_config_t motor_3_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = MOTOR_3_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_3_GPIO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_3_channel));

    ledc_channel_config_t motor_4_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = MOTOR_4_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_4_GPIO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_4_channel));
}

void update_pwm(motor_cmds_t motor_cmds) {
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, MOTOR_1_CHANNEL, DUTY_CYCLE_PCT_2_VAL(motor_cmds.motor1_duty_cycle_pct)));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, MOTOR_1_CHANNEL));
    
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, MOTOR_2_CHANNEL, DUTY_CYCLE_PCT_2_VAL(motor_cmds.motor2_duty_cycle_pct)));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, MOTOR_2_CHANNEL));
    
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, MOTOR_3_CHANNEL, DUTY_CYCLE_PCT_2_VAL(motor_cmds.motor3_duty_cycle_pct)));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, MOTOR_3_CHANNEL));
    
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, MOTOR_3_CHANNEL, DUTY_CYCLE_PCT_2_VAL(motor_cmds.motor4_duty_cycle_pct)));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, MOTOR_3_CHANNEL));
}