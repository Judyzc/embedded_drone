#ifndef SENSORS_H
#define SENSORS_H

#include "driver/i2c_master.h"

/* ------------------------------------------- Structs ------------------------------------------- */
typedef struct acc_data {
    float ax_m_s2; 
    float ay_m_s2; 
    float az_m_s2; 
} acc_data_t;

typedef struct gyro_data {
    float Gx_rad_s; 
    float Gy_rad_s; 
    float Gz_rad_s; 
} gyro_data_t;

/* ------------------------------------------- Constants ------------------------------------------- */
// I2C addresses and device registers 
#define IMU_ACC_SENSOR_ADDR         0x18                        /* Accelerometer I2C Address */
#define ACC_PWR_CTRL                0x7D                        /* Accelerometer registers */
#define ACC_CONF                    0x40
#define ACC_RANGE                   0x41
#define ACC_DATA_START              0x12

#define IMU_GYRO_SENSOR_ADDR        0x69                        /* Gyroscope I2C Address */
#define GYRO_LPM1                   0x11                        /* Gyroscope registers */
#define GYRO_RANGE                  0x0F
#define GYRO_BANDWIDTH              0x10
#define GYRO_DATA_START             0x02

#define TOF_SENSOR_ADDR             0x29                        /* Time of Flight I2C Address*/

#define ESP_SCLK_IO 18  // 3, right side of deck
#define ESP_MOSI_IO 23  // 5, right side of deck
#define ESP_MISO_IO 19  // 4, right side of deck
#define ESP_CS_IO   5   // 8, left side of deck


// General Constants
#define GET_RAW_DATA_PRIORITY       6
#define DATA_PROC_PRIORITY          5
#define SENS_PERIOD_MS              4                           /* Sensor polling rate during stabilization loop */
#define TOF_SENS_PERIOD_MS          50                          /* Data rate of ToF sensor */
#define CALIBRATION_PERIOD_MS       2                           /* Sensor polling rate during calibration */
#define CALIBRATION_SAMPLES         1500                        /* Number of samples to take while calibrating sensors */

/* ------------------------------------------- Public Global Variables  ------------------------------------------- */
extern i2c_master_dev_handle_t tof_handle;
extern float ave_g_m_s2; 

/* ------------------------------------------- Public Function Declarations ------------------------------------------- */
void sensors_init(void);
void calibrate_sensors(void);
void start_control_loop(void);
void init_osc_pin(int pin);

#endif /* SENSORS_H */