#include <stdio.h>

#ifndef HELPER_H
#define HELPER_H

int64_t get_time(void);

void init_osc_pin(int pin);

float rad2deg(float r);
float deg2rad(float d);

#endif