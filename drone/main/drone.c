#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
// logs
#include <esp_log.h>
// motors pwm
#include <driver/mcpwm.h>
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
// reference code: https://github.com/espressif/esp-idf/blob/b9c6175649647580cfa9ae33432e53ff2ccd2ca1/examples/peripherals/mcpwm/mcpwm_brushed_dc_control/main/mcpwm_brushed_dc_control_example.c 

#define LED_PIN 2
#define GPIO_PWM0A_OUT 23
static const char *TAG = "test: ";


static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;

    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
}


static void motor_control(void *arg) {
    ESP_LOGI(TAG, "init motor\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);

    ESP_LOGI(TAG, "initializing motor params\n");
    mcpwm_config_t motor_config = {
        .frequency = 1000, // 1 kHz
        .cmpr_a = 0.0f, // duty cycle
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_config);

    while (1) {
        ESP_LOGI(TAG, "forward 50%%\n");
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 80.0);
        vTaskDelay(pdMS_TO_TICKS(3000));
        gpio_set_level(LED_PIN, 1);

        ESP_LOGI(TAG, "stop\n");
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(pdMS_TO_TICKS(3000));
        gpio_set_level(LED_PIN, 0);
    }
}

void app_main() {
    // Configure the LED pin (debugging)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // brushed coreless dc motors
    xTaskCreate(motor_control, "motor_control", 4096, NULL, 5, NULL);
}