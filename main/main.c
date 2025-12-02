#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

#include "i2c_setup.h"
#include "spi_setup.h"
#include "sensors.h"
#include "estimator.h"
#include "controllers.h"
#include "motors.h"
#include "espnow.h"
#include "digital_io.h"
#include "bmi088.h"
#include "pmw3901.h"

static const char *TAG = "main";

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void app_main(void) {
    // Initialize Queues between tasks
    QueueHandle_t xQueue_acc_data = xQueueCreate(1, sizeof(acc_data_t)); 
    QueueHandle_t xQueue_gyro_data = xQueueCreate(1, sizeof(gyro_data_t)); 
    QueueHandle_t xQueue_state_data = xQueueCreate(1, sizeof(state_data_t)); 
    QueueHandle_t xQueue_tof_data = xQueueCreate(1, sizeof(uint16_t)); 
    QueueHandle_t xQueue_opt_flow_data = xQueueCreate(1, sizeof(motionBurst_t));
    
    // Call initialization functions
    ESP_LOGI(TAG, "Initializing components");

    init_osc_pin(CONFIG_PIN_TOGGLE_A);
    init_osc_pin(CONFIG_PIN_TOGGLE_B);

    i2c_master_bus_handle_t bus_handle;
    i2c_master_init(&bus_handle); 

    spi_master_init(VSPI_HOST); 

    sensors_init(&bus_handle, &xQueue_acc_data, &xQueue_gyro_data, &xQueue_tof_data, &xQueue_opt_flow_data); 
    estimator_init(&xQueue_acc_data, &xQueue_gyro_data, &xQueue_tof_data, &xQueue_opt_flow_data, &xQueue_state_data); 
    controllers_init(&xQueue_state_data);
    motors_init(); 

    // Calibrate sensors and start stabilization loop
    calibrate_IMU();
    for (int i=2; i>0; i--) {
        ESP_LOGI(TAG, "Starting in: %d", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    start_control_loop();

    // test_wifi();

    while (1) {
        vTaskDelay(1);      // Prevent watchdog from timing out
    }

    ESP_LOGE(TAG, "Exited main loop"); 
}