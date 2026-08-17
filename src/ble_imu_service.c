#include "ble_imu_service.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* Custom 128-bit UUIDs — generated for this project, keep as-is or regenerate your own */
#define BT_UUID_IMU_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abc0001)
#define BT_UUID_IMU_BATCH_CHRC_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abc0002)

static struct bt_uuid_128 imu_svc_uuid   = BT_UUID_INIT_128(BT_UUID_IMU_SERVICE_VAL);
static struct bt_uuid_128 imu_batch_uuid = BT_UUID_INIT_128(BT_UUID_IMU_BATCH_CHRC_VAL);

static struct bt_conn *imu_conn;
static bool notify_enabled;

/* Double buffer so we can fill one while the other is in-flight over BLE */
static struct imu_batch_packet batch_buf[2];
static uint8_t active_buf = 0;
static uint8_t sample_idx = 0;

static void imu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("[BLE IMU] Notifications %s\n", notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(imu_svc,
    BT_GATT_PRIMARY_SERVICE(&imu_svc_uuid),
    BT_GATT_CHARACTERISTIC(&imu_batch_uuid.uuid,
        BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(imu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void imu_connected(struct bt_conn *conn, uint8_t err)
{
    if (!err) {
        imu_conn = conn;
    }
}

static void imu_disconnected(struct bt_conn *conn, uint8_t reason)
{
    imu_conn = NULL;
    notify_enabled = false;
}

BT_CONN_CB_DEFINE(imu_conn_callbacks) = {
    .connected = imu_connected,
    .disconnected = imu_disconnected,
};

bool ble_imu_is_connected(void)
{
    return imu_conn != NULL && notify_enabled;
}

int ble_imu_service_init(void)
{
    sample_idx = 0;
    active_buf = 0;
    memset(batch_buf, 0, sizeof(batch_buf));
    return 0;
}

static int16_t sv_to_millis(const struct sensor_value *v)
{
    double d = sensor_value_to_double(v) * 1000.0;
    if (d > 32767.0) d = 32767.0;
    if (d < -32768.0) d = -32768.0;
    return (int16_t)d;
}

static void flush_batch(uint8_t buf_to_send)
{
    if (!ble_imu_is_connected()) {
        return;
    }

    int err = bt_gatt_notify(imu_conn, &imu_svc.attrs[1],
                              &batch_buf[buf_to_send],
                              sizeof(struct imu_batch_packet));
    if (err) {
        printk("[BLE IMU] notify failed err=%d (buffer likely full)\n", err);
    }
}

void ble_imu_batch_add_sample(const struct sensor_value *acc,
                               const struct sensor_value *gyro)
{
    struct imu_batch_packet *pkt = &batch_buf[active_buf];

    if (sample_idx == 0) {
        pkt->batch_start_ts = (uint32_t)k_uptime_get();
    }

    struct imu_sample_packed *s = &pkt->samples[sample_idx];
    s->ax = sv_to_millis(&acc[0]);
    s->ay = sv_to_millis(&acc[1]);
    s->az = sv_to_millis(&acc[2]);
    s->gx = sv_to_millis(&gyro[0]);
    s->gy = sv_to_millis(&gyro[1]);
    s->gz = sv_to_millis(&gyro[2]);

    sample_idx++;

    if (sample_idx >= IMU_BATCH_SIZE) {
        uint8_t full_buf = active_buf;
        active_buf = 1 - active_buf;   // swap to the other buffer immediately
        sample_idx = 0;
        flush_batch(full_buf);
    }
}