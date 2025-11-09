#ifndef SENSORS_H
#define SENSORS_H
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
// I2C Config info
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

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

// General Constants
#define GET_RAW_DATA_PRIORITY       6
#define DATA_PROC_PRIORITY          5
#define SENS_PERIOD_MS              2                           /* Sensor polling rate during stabilization loop */
#define CALIBRATION_PERIOD_MS       2                           /* Sensor polling rate during calibration */
#define CALIBRATION_SAMPLES         3000                        /* Number of samples to take while calibrating sensors */

/* ------------------------------------------- Public Function Declarations ------------------------------------------- */
void sensors_init(void);
void calibrate_sensors(void);
void start_control_loop(void);

#endif /* SENSORS_H */