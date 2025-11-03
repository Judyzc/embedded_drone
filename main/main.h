#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Global variables
extern QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_state_data, xQueue_ToF_data; 

#endif /* MAIN_H */