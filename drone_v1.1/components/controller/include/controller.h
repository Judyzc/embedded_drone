#include "complementary_filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#ifndef CONTROLLER_H
#define CONTROLLER_H

extern TickType_t start_tick, end_tick;

typedef struct {
    int64_t timestamp; // us
    float cmd;
} controller_cmd_t;

void controller_update(estimated_state_t *estimated_state);

void controller_init(void); 

#endif
