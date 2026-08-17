#ifndef SENSOR_DATA_COLLECTOR_H
#define SENSOR_DATA_COLLECTOR_H     

#include <zephyr/drivers/sensor.h>

typedef struct {
    struct sensor_value temp;
    struct sensor_value pres;
    struct sensor_value humidity;
    struct sensor_value gas_res;
    struct sensor_value acc[3]; // Assuming 3-axis accelerometer
    struct sensor_value gyro[3]; // Assuming 3-axis gyroscope
} Sensorreadings;

#endif // SENSOR_DATA_COLLECTOR_H