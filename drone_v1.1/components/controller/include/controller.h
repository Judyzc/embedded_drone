#include "complementary_filter.h"

#ifndef CONTROLLER_H
#define CONTROLLER_H

typedef struct {
    int64_t timestamp; // us
    float cmd;
} controller_cmd_t;

void controller_update(attitude_t *att, controller_cmd_t *control_cmd);


#endif