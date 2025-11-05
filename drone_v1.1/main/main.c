#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "complementary_filter.h"
#include "sensors.h"
#include "helper.h"
#include "i2c.h"
#include "bmp3.h"
#include "bmp3_defs.h"
#include "controller.h"
#include <math.h>

// task pins for oscilloscope
#define PIN_IMU_TOGGLE  2    
#define PIN_BARO_TOGGLE 4    
#define PIN_TOF_TOGGLE  12
#define PIN_FILTER_TOGGLE 14
#define PIN_CTRL_TOGGLE 5   

#define SAMPLE_HZ 50
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_HZ) // 10 ms sample period

#define QUEUE_LEN 1

static const char *TAG = "MAIN";

static QueueHandle_t imu_q;
static QueueHandle_t baro_q;
static QueueHandle_t tof_q;
static QueueHandle_t attitude_q;

static estimated_state_t estimated_state;

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev_handle;

static void imu_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();
    imu_sample_t sample;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        gpio_set_level(PIN_IMU_TOGGLE, 1);

        // read sensor (stub)
        mock_imu_read(&sample);

        // publish latest sample (single-slot queue)
        xQueueOverwrite(imu_q, &sample);

        // esp_rom_delay_us(10000); // delay 10 ms
        
        gpio_set_level(PIN_IMU_TOGGLE, 0);
    }
}

static void baro_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();
    baro_sample_t sample;

    uint8_t who = 0;
    if (i2c_master_read_reg(dev_handle, 0x00, &who, 1, 100) == ESP_OK) {
        ESP_LOGI(TAG, "BMP388 WHO_AM_I = 0x%02X", who);
    }

    while (1) {
        // ensures that the task will run at period
        // starts with current time + period, continues from there
        vTaskDelayUntil(&last_wake, period);
        gpio_set_level(PIN_BARO_TOGGLE, 1);

        baro_read(&sample);
        
        xQueueOverwrite(baro_q, &sample);
        ESP_LOGI(TAG, "BMP388 temp=%.2f, pressure=%.2f, asl=%.2f, time=%d", sample.temperature, sample.pressure, sample.asl, (sample.timestamp / 1000));

        gpio_set_level(PIN_BARO_TOGGLE, 0);
    }
}

static void tof_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();
    tof_sample_t sample;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        gpio_set_level(PIN_TOF_TOGGLE, 1);

        mock_tof_read(&sample);
        
        xQueueOverwrite(tof_q, &sample);

        gpio_set_level(PIN_TOF_TOGGLE, 0);
    }
}

static void filter_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    cf_init(&estimated_state);

    imu_sample_t imu_s;
    baro_sample_t baro_s;
    tof_sample_t tof_s;

    // ToF
    float altitude_mm = 0.0f;
    const float alt_alpha = 0.6f;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        gpio_set_level(PIN_FILTER_TOGGLE, 1);

        // Try to read latest samples (non-blocking)
        if (xQueuePeek(imu_q, &imu_s, 0) == pdFALSE) {
            // no imu sample - skip update
            gpio_set_level(PIN_FILTER_TOGGLE, 0);
            continue;
        }

        xQueuePeek(tof_q, &tof_s, 0);

        xQueuePeek(baro_q, &baro_s, 0);

        ESP_LOGI(TAG, "BMP altitude=%.2f, time=%d", baro_s.asl, (baro_s.timestamp / 1000));

        // update complementary filter
        cf_update(&estimated_state, &imu_s, &tof_s);

        // altitude fusion: simple low-pass on ToF
        if (tof_s.distance_mm > 0.0f) {
            if (altitude_mm == 0.0f) altitude_mm = tof_s.distance_mm;
            altitude_mm = alt_alpha * altitude_mm + (1.0f - alt_alpha) * tof_s.distance_mm;
        }

        // publish attitude
        attitude_t att = {
            .timestamp = get_time(),
            .roll = estimated_state.roll,
            .pitch = estimated_state.pitch,
            .yaw = estimated_state.yaw,
            .altitude_mm = altitude_mm
        };

        xQueueOverwrite(attitude_q, &att);

        ESP_LOGI(TAG, "att: roll=%.2f pitch=%.2f altitude=%.1f", att.roll, att.pitch, att.altitude_mm);

        gpio_set_level(PIN_FILTER_TOGGLE, 0);
    }
}

static void controller_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    attitude_t att;
    controller_cmd_t control_cmd;
    while (1) {
        vTaskDelayUntil(&last_wake, period);

        gpio_set_level(PIN_CTRL_TOGGLE, 1);

        // read latest attitude (non-blocking)
        if (xQueuePeek(attitude_q, &att, 0) == pdTRUE) {
            controller_update(&att, &control_cmd);
            // Dummy controller: for now just log
            ESP_LOGI(TAG, "controller saw att: r=%.2f p=%.2f alt=%.1f", att.roll, att.pitch, att.altitude_mm);
        }

        // esp_rom_delay_us(10000);

        gpio_set_level(PIN_CTRL_TOGGLE, 0);
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting drone!");

    // initialize pins
    init_osc_pin(PIN_IMU_TOGGLE);
    init_osc_pin(PIN_BARO_TOGGLE);
    init_osc_pin(PIN_TOF_TOGGLE);
    init_osc_pin(PIN_FILTER_TOGGLE);
    init_osc_pin(PIN_CTRL_TOGGLE);

    ESP_ERROR_CHECK(i2c_master_init_bus(-1, GPIO_NUM_21, GPIO_NUM_22, 400000, &bus));

    // add BMP388 to I2C bus
    ESP_ERROR_CHECK(i2c_master_add_device(bus, 0x77, &dev_handle));

    // initialize BMP388static void sensorsDeviceInit(void)
    baro_init(&dev_handle);

    // Create queues (single-slot)
    imu_q = xQueueCreate(QUEUE_LEN, sizeof(imu_sample_t));
    baro_q = xQueueCreate(QUEUE_LEN, sizeof(baro_sample_t));
    tof_q = xQueueCreate(QUEUE_LEN, sizeof(tof_sample_t));
    attitude_q = xQueueCreate(QUEUE_LEN, sizeof(attitude_t));

    if (!imu_q || !baro_q || !tof_q || !attitude_q) {
        ESP_LOGE(TAG, "Failed to create queues");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // core 0
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(baro_task, "baro_task", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(tof_task, "tof_task", 4096, NULL, 2, NULL, 0);

    // core 1
    xTaskCreatePinnedToCore(filter_task, "filter_task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(controller_task, "controller_task", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "Tasks started");
}