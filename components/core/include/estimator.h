#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "sensors.h"
#include "math.h"

/* ------------------------------------------- Structs ------------------------------------------- */
typedef struct state_data {
    float pitch_rad; 
    float pitch_rate_rad_s; 
    float roll_rad; 
    float roll_rate_rad_s; 
    float yaw_rate_rad_s; 
    float altitude_m; 
    float altitude_rate_m_s; 
    float vel_x_m_s; 
    float vel_y_m_s;
} state_data_t;

/* ------------------------------------------- Constants  ------------------------------------------- */
// Complementary filter parameters
#define TAU                     1.0f
#define DELTA_T                 (((float) SENS_PERIOD_MS)*.001f)

// General Constants
#define ESTIMATOR_PRIORITY      4
#define OPT_FLOW_FOV_RAD        (42.0f*M_PI/180.0f)
#define OPT_FLOW_PX_LENGTH      30

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void estimator_init(
    QueueHandle_t *pxQueue_acc_data, 
    QueueHandle_t *pxQueue_gyro_data,
    QueueHandle_t *pxQueue_tof_data,
    QueueHandle_t *pxQueue_opt_flow_data,
    QueueHandle_t *pxQueue_state_data
);

#endif /* ESTIMATOR_H */