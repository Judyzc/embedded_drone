#include "driver/gpio.h"
#include "helper.h"
#include "esp_timer.h"

int64_t get_time(void) {
    return esp_timer_get_time();
}

void init_osc_pin(int pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(pin, 0);
}

float rad2deg(float r) { return r * 57.29577951308232f; }
float deg2rad(float d) { return d * 0.017453292519943295f; }