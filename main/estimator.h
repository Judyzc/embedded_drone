#ifndef ESTIMATOR_H
#define ESTIMATOR_H

// Public function declarations
void estimator_init(QueueHandle_t xQueue_acc_data, QueueHandle_t xQueue_gyro_data);

// Complementary filter parameters
#define TAU                     1.0f

// Constants
#define ESTIMATOR_PRIORITY      4

#endif /* ESTIMATOR_H */