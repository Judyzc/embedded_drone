#ifndef BMI088_H
#define BMI088_H

#include "esp_err.h"
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

// General constants
#define IMU_CALIBRATION_PERIOD_MS   2                           /* Sensor polling rate during calibration */
#define CALIBRATION_SAMPLES         1500                        /* Number of samples to take while calibrating sensors */

/* ------------------------------------------- Public Global Variables ------------------------------------------- */
extern float ave_g_m_s2; 

/* ------------------------------------------- Public Function Declarations ------------------------------------------- */
esp_err_t IMU_acc_init(i2c_master_bus_handle_t *bus_handle);  
esp_err_t IMU_gyro_init(i2c_master_bus_handle_t *bus_handle);
void calibrate_IMU(void);
acc_data_t get_acc_data(void);
gyro_data_t get_gyro_data(void);
void calibrate_IMU(void); 

#endif /* BMI088_H */