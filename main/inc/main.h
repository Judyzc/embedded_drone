#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ------------------------------------------- Global Variables  ------------------------------------------- */
extern QueueHandle_t xQueue_acc_data, xQueue_gyro_data, xQueue_state_data, xQueue_ToF_data, xQueue_optf_data; 

#define PIN_TOGGLE_A    25
#define PIN_TOGGLE_B    10

#endif /* MAIN_H */