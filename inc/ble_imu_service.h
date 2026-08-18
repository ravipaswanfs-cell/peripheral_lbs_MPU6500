#ifndef BLE_IMU_SERVICE_H
#define BLE_IMU_SERVICE_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdbool.h>

#define IMU_BATCH_SIZE  10

struct imu_sample_packed{
    int16_t ax,ay,az; //mili-g
    int16_t gx,gy,gz; // mili dps
}__packed;

struct imu_batch_packet{
    uint32_t batch_start_ts;
    struct imu_sample_packed samples[IMU_BATCH_SIZE];
}__packed;


int ble_imu_service_init(void);
void ble_imu_batch_add_sample(const struct sensor_value *acc,
                               const struct sensor_value *gyro);

bool ble_imu_is_connected(void);

#endif

