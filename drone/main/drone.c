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
static const char *TAG = "MOTORS: ";


#include "driver/i2c.h"
#define I2C_MASTER_NUM I2C_NUM_0 // not sure if actual 
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define VL53L0X_ADDR 0x29
static const char *TOF_TAG = "VL53L0X: ";


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


static void tof_control() {
    while (1) {
        // Placeholder for distance measurement implementation
        ESP_LOGI(TAG, "Reading distance...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}



void app_main() {
    // Configure the LED pin (just debugging)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // contifure motors 
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

    // configure tof 
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TOF_TAG, "VL53L0X initialized.");

    // brushed coreless dc motors
    xTaskCreate(motor_control, "motor_control", 4096, NULL, 5, NULL);
    // tof 
    xTaskCreate(tof_control, "tof_control", 4096, NULL, 5, NULL);
}