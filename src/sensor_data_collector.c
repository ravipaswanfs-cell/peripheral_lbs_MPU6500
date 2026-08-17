#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nsms.h>
#include <zephyr/devicetree.h>
#include "sensor_data_collector.h"
#include "ble_imu_service.h"


#define SENSOR_THREAD_PRIORITY 7
#define SENSOR_THREAD_STACK_SIZE 1024
#define BUF_SIZE 64

BT_NSMS_DEF(nsms_imu, "IMU", false,"Unknown", BUF_SIZE);
BT_NSMS_DEF(nsms_env, "Environmental", false,"Unknown", BUF_SIZE);

static struct bt_conn *current_conn = NULL;

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
    } else {
        printk("Connected\n");
        current_conn = bt_conn_ref(conn);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};


static bool send_sensor_value(const struct sensor_value *val, size_t size, const char *channel){

    float float_data[size];
    char buf[BUF_SIZE];

    int len = snprintf(buf, BUF_SIZE, "%s", channel);

    for (size_t i = 0; i < size ; i++){

        float_data[i] = sensor_value_to_float(&val[i]);

        int rem = BUF_SIZE - len;
        if (rem <= 0) {
            break;
        }
        int n = snprintf(buf + len, rem, " %.6f", float_data[i]);
        if (n < 0) {
            break;
        }
        len += n;

    }

    if(!strcmp(channel,"gyr")){
        bt_nsms_set_status(&nsms_imu, buf);
    }
    else if(!strcmp(channel,"acc")){
        bt_nsms_set_status(&nsms_env, buf);
    } else{
        bt_nsms_set_status(&nsms_env, buf);
    }
    return true;
}


static const struct device *get_mpu6500_sensor(void){
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(mpu6500));

    if (dev == NULL) {
        printk("Sensor device not found\n");
        return NULL;
    }

    if (!device_is_ready(dev)) {
        printk("Sensor device not ready\n");
        return NULL;
    }

    printk("Found Device \"%s\" ,getting sensor data\n", dev->name);

    return dev;
}

int sensor_data_collection_init(void) {

    const struct device *sim_dev = get_mpu6500_sensor();

    if (sim_dev == NULL) {
        printk("[INIT FAIL] MPU6500 device not found/ready\n");
        return -1;
    }

    int64_t last_ts = k_uptime_get();
    int64_t max_dt = 0;
    int64_t sample_count = 0;

    while (1) {
        Sensorreadings value;
        int rc = sensor_sample_fetch(sim_dev);
        if (rc) {
            printk("[SAMPLE FETCH FAIL] err=%d, retrying in %dms\n",
                   rc, CONFIG_APP_SAMPLING_INTERVAL_MS);
            k_sleep(K_MSEC(CONFIG_APP_SAMPLING_INTERVAL_MS));
            continue;
        }

        int64_t now = k_uptime_get();
        int64_t dt = now - last_ts;
        last_ts = now;
        sample_count++;
        if (dt > max_dt) {
            max_dt = dt;
        }

        sensor_channel_get(sim_dev, SENSOR_CHAN_DIE_TEMP, &value.temp);
        sensor_channel_get(sim_dev, SENSOR_CHAN_ACCEL_XYZ, value.acc);
        sensor_channel_get(sim_dev, SENSOR_CHAN_GYRO_XYZ, value.gyro);

        printk("[SAMPLE #%lld] dt=%lldms (target=%dms, worst=%lldms)%s\n",
               sample_count, dt, CONFIG_APP_SAMPLING_INTERVAL_MS, max_dt,
               (dt > CONFIG_APP_SAMPLING_INTERVAL_MS + 5) ? "  <-- JITTER SPIKE" : "");

        printk("  Temp: %.2f C | Accel[X:%.2f Y:%.2f Z:%.2f] | Gyro[X:%.2f Y:%.2f Z:%.2f]\n",
               value.temp.val1 + value.temp.val2 / 1000000.0,
               value.acc[0].val1 + value.acc[0].val2 / 1000000.0,
               value.acc[1].val1 + value.acc[1].val2 / 1000000.0,
               value.acc[2].val1 + value.acc[2].val2 / 1000000.0,
               value.gyro[0].val1 + value.gyro[0].val2 / 1000000.0,
               value.gyro[1].val1 + value.gyro[1].val2 / 1000000.0,
               value.gyro[2].val1 + value.gyro[2].val2 / 1000000.0);


    ble_imu_batch_add_sample(value.acc, value.gyro);

    if (current_conn != NULL) {
        send_sensor_value(&value.temp, 1, "temp");   // keep temp on NSMS, it's slow-changing
    }

        k_sleep(K_MSEC(CONFIG_APP_SAMPLING_INTERVAL_MS));
    }
    return 0;
}

K_THREAD_DEFINE(sensor_data_collector_id, SENSOR_THREAD_STACK_SIZE, sensor_data_collection_init, NULL, NULL, NULL, SENSOR_THREAD_PRIORITY, 0, 1000);
