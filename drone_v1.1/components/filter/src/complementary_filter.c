#include <stdio.h>
#include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "esp_system.h"
#include "complementary_filter.h"
#include "helper.h"
#include <math.h>

void cf_init(estimated_state_t *s) {
    memset(s, 0, sizeof(*s));
    s->timestamp = get_time();
}

void cf_update(estimated_state_t *s, imu_sample_t *imu, tof_sample_t *tof) {
    if (!imu) return;
    int64_t now = imu->timestamp;
    float dt = (now - s->timestamp) / 1e6f;
    if (dt <= 0 || dt > 0.5f) dt = 0.05f; // guard

    // Integrate gyro rates (deg/s) -> deg
    s->roll += imu->gyro[0] * dt;   // deg
    s->pitch += imu->gyro[1] * dt;  // deg
    s->yaw += imu->gyro[2] * dt;    // deg

    // Accel-based roll/pitch estimate (small-angle-safe)
    // roll = atan2(accel_y, accel_z)
    float acc_roll = rad2deg(atan2f(imu->accel[1], imu->accel[2]));
    float acc_pitch = rad2deg(atan2f(-imu->accel[0], sqrtf(imu->accel[1]*imu->accel[1] + imu->accel[2]*imu->accel[2])));

    // Complementary fusion
    const float alpha = 0.98f; // gyro weight
    s->roll  = alpha * s->roll  + (1.0f - alpha) * acc_roll;
    s->pitch = alpha * s->pitch + (1.0f - alpha) * acc_pitch;

    s->timestamp = now;
}