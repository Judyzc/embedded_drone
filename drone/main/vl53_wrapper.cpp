// vl53_wrapper.cpp - minimal C++ wrapper that exposes a C API
#include "VL53L0X.h"
#include "esp_log.h"
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "VL53_WR";

// single shared driver instance (nullptr until initialized)
static VL53L0X *vl = nullptr;

extern "C" {

/**
 * Initialize the driver's I2C master.
 * i2c_port is ignored by the kerikun11 driver constructor (it accepts port),
 * but keep parameter for API similarity. Returns true on success.
 */
bool vl53_i2c_init(int i2c_port, int pin_sda, int pin_scl)
{
    (void)i2c_port; // unused in this wrapper (kept for API clarity)
    // allocate VL53L0X object on heap
    if (vl) {
        // already initialized
        return true;
    }
    vl = new VL53L0X((i2c_port_t)I2C_NUM_0);
    if (!vl) {
        ESP_LOGE(TAG, "Failed to new VL53L0X");
        return false;
    }
    vl->i2cMasterInit((gpio_num_t)pin_sda, (gpio_num_t)pin_scl);
    return true;
}

/** Initialize the sensor (runs calibration). Returns true on success. */
bool vl53_init(void)
{
    if (!vl) {
        ESP_LOGE(TAG, "vl53_init called but wrapper not initialized");
        return false;
    }
    bool ok = vl->init();
    if (!ok) {
        ESP_LOGW(TAG, "VL53L0X::init() returned false");
    }
    return ok;
}

/** Read a single range measurement (blocking). Returns true on success and sets distance_mm. */
bool vl53_read(uint16_t *distance_mm)
{
    if (!vl || !distance_mm) return false;
    bool ok = vl->read(distance_mm);
    return ok;
}

/** Deinitialize and free resources */
void vl53_deinit(void)
{
    if (vl) {
        delete vl;
        vl = nullptr;
    }
}

} // extern "C"
