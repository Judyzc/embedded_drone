#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
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
#include "motor.h"

static const char *TAG = "MAIN";

#define SAMPLE_HZ 50
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_HZ) // 10 ms sample period

#define QUEUE_LEN 1

static QueueHandle_t acc_q;
static QueueHandle_t gyro_q;
static QueueHandle_t baro_q;
static QueueHandle_t tof_q;
static QueueHandle_t state_q;

static estimated_state_t estimated_state;

static void imu_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    acc_sample_t acc_sample;
    gyro_sample_t gyro_sample;

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // read sensor (stub)
        acc_read(&acc_sample);
        xQueueOverwrite(acc_q, &acc_sample);

        gyro_read(&gyro_sample);
        xQueueOverwrite(gyro_q, &gyro_sample);

        ESP_LOGI(TAG, "ACC ax=%x, ay=%x, az=%x", acc_sample.ax, acc_sample.ay, acc_sample.az);

        ESP_LOGI(TAG, "GYRO gx=%.2f, gy=%.2f, gz=%.2f", gyro_sample.gx, gyro_sample.gy, gyro_sample.gz);
    }
}

static void baro_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();
    baro_sample_t sample;

    uint8_t who = 0;
    if (i2c_master_read_reg(baro_handle, 0x00, &who, 1, 100) == ESP_OK) {
        ESP_LOGI(TAG, "BMP388 WHO_AM_I = 0x%02X", who);
    }

    while (1) {
        // ensures that the task will run at period
        // starts with current time + period, continues from there
        vTaskDelayUntil(&last_wake, period);

        baro_read(&sample);
        
        xQueueOverwrite(baro_q, &sample);
        ESP_LOGI(TAG, "BMP388 temp=%.2f, pressure=%.2f, asl=%.2f, time=%d", sample.temperature, sample.pressure, sample.asl, (sample.timestamp / 1000));
    }
}

static void tof_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();
    tof_sample_t sample;

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        mock_tof_read(&sample);
        
        xQueueOverwrite(tof_q, &sample);
    }
}

static void estimator_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    acc_sample_t acc_s;
    gyro_sample_t gyro_s;
    baro_sample_t baro_s;
    // tof_sample_t tof_s;

    // ToF
    // float altitude_mm = 0.0f;
    // const float alt_alpha = 0.6f;

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        xQueuePeek(acc_q, &acc_s, 0);
       
        xQueuePeek(gyro_q, &gyro_s, 0);

        xQueuePeek(baro_q, &baro_s, 0);



        // xQueuePeek(tof_q, &tof_s, 0);


        ESP_LOGI(TAG, "BMP altitude=%.2f, time=%d", baro_s.asl, (baro_s.timestamp / 1000));

        // update complementary filter
        estimator_update(&estimated_state, &acc_s, &gyro_s, &baro_s);

        xQueueOverwrite(state_q, &estimated_state);

        ESP_LOGI(TAG, "Estimated state: roll=%.2f pitch=%.2f altitude=%.1f, roll_rate=%.2f, pitch_rate=%.2f, yaw_rate=%.2f", estimated_state.roll, estimated_state.pitch, estimated_state.altitude, estimated_state.roll_rate, estimated_state.pitch_rate, estimated_state.yaw_rate);
    }
}

static void controller_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    estimated_state_t estimated_state;
    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // read latest attitude (non-blocking)
        if (xQueuePeek(state_q, &estimated_state, 0) == pdTRUE) {
            controller_update(&estimated_state);
        }
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting drone!");

    sensors_init();
    estimator_init();
    controller_init();
    motors_init();

    // Create queues (single-slot)
    acc_q = xQueueCreate(1, sizeof(acc_sample_t)); 
    gyro_q = xQueueCreate(1, sizeof(gyro_sample_t)); 
    // state_q = xQueueCreate(1, sizeof(state_data_t)); 

    baro_q = xQueueCreate(QUEUE_LEN, sizeof(baro_sample_t));
    tof_q = xQueueCreate(QUEUE_LEN, sizeof(tof_sample_t));
    state_q = xQueueCreate(QUEUE_LEN, sizeof(estimated_state_t));

    if (!acc_q || !gyro_q || !baro_q || !tof_q || !state_q) {
        ESP_LOGE(TAG, "Failed to create queues");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // core 0
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(baro_task, "baro_task", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(tof_task, "tof_task", 4096, NULL, 2, NULL, 0);

    // core 1
    xTaskCreatePinnedToCore(estimator_task, "filter_task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(controller_task, "controller_task", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "Tasks started");
}