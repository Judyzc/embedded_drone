#ifndef SENSORS_H
#define SENSORS_H

#include "driver/i2c_master.h"

/* ------------------------------------------- Constants ------------------------------------------- */
#define GET_RAW_DATA_PRIORITY       6
#define SENS_PERIOD_MS              2                           /* Sensor polling rate during stabilization loop */

/* ------------------------------------------- Public Function Declarations ------------------------------------------- */
void sensors_init(
    i2c_master_bus_handle_t *bus_handle, 
    QueueHandle_t *pxQueue_acc_data, 
    QueueHandle_t *pxQueue_gyro_data,
    QueueHandle_t *pxQueue_tof_data,
    QueueHandle_t *pxQueue_opt_flow_data
);
void start_control_loop(void);

#endif /* SENSORS_H */