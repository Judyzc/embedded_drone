#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "pid_ctrl.h"

#include "main.h"
#include "sensors.h"
#include "six_axis_comp_filter.h"
#include "estimator.h"
#include "controllers.h"

static const char *TAG = "controllers";

/* ------------------------------------------- Private Function Definitions  ------------------------------------------- */
void vUpdatePIDTask(void *pvParameters) {

}

/* ------------------------------------------- Public Function Definitions  ------------------------------------------- */
void controllers_init(void) {
    // Start filter update task
    // xTaskCreate(vUpdatePIDTask, "Cascaded PID", 4096, NULL, ESTIMATOR_PRIORITY, NULL);
}