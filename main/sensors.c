#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "esp_system.h"
#include <math.h>



void mock_imu_read(imu_sample_t *imu_sample) {
    imu_sample->timestamp = get_time();
    // simple synthetic motion: slow oscillation so filter shows something
    static float t = 0.0f;
    t += 0.05f;

    imu_sample->accel[0] = 0.0f; // X
    imu_sample->accel[1] = 0.0f; // Y
    imu_sample->accel[2] = 9.81f; // Z (gravity)

    // pretend a slow roll rate about X and pitch about Y (deg/s)
    imu_sample->gyro[0] = 5.0f * sinf(t); // roll rate
    imu_sample->gyro[1] = 3.0f * cosf(t); // pitch rate
    imu_sample->gyro[2] = 0.0f; // yaw rate
}

void mock_baro_read(baro_sample_t *baro_sample) {
    baro_sample->timestamp = get_time();
    static float alt = 1000.0f;
    // small noise
    alt += 0.1f;
    baro_sample->pressure = 101325.0f - alt; // arbitrary mapping
    baro_sample->temperature = 25.0f;
}

void mock_tof_read(tof_sample_t *tof_sample) {
    tof_sample->timestamp = get_time();
    static float d = 500.0f; // mm
    d += 0.2f;
    tof_sample->distance_mm = d;
}
