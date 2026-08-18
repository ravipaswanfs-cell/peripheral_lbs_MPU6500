#include "ble_imu_service.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* Custom 128-bit UUIDs for this project. Regenerate your own if you want
 * globally unique values (e.g. `uuidgen`), these are fine for dev/test. */
#define BT_UUID_IMU_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abc0001)
#define BT_UUID_IMU_BATCH_CHRC_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abc0002)

static struct bt_uuid_128 imu_svc_uuid   = BT_UUID_INIT_128(BT_UUID_IMU_SERVICE_VAL);
static struct bt_uuid_128 imu_batch_uuid = BT_UUID_INIT_128(BT_UUID_IMU_BATCH_CHRC_VAL);

static struct bt_conn *imu_conn;
static bool notify_enabled;

/* Double buffer: fill one while the other is (potentially) still being
 * transmitted, so filling never blocks on the radio. */
static struct imu_batch_packet batch_buf[2];
static uint8_t active_buf;
static uint8_t sample_idx;

static uint32_t notify_ok_count;
static uint32_t notify_fail_count;

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
        notify_ok_count = 0;
        notify_fail_count = 0;
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
    notify_ok_count = 0;
    notify_fail_count = 0;
    return 0;
}

static int16_t sv_to_millis(const struct sensor_value *v)
{
    double d = sensor_value_to_double(v) * 1000.0;
    if (d > 32767.0) {
        d = 32767.0;
    }
    if (d < -32768.0) {
        d = -32768.0;
    }
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
        notify_fail_count++;
        /* Throttle failure logging so a jammed link doesn't itself flood RTT */
        if (notify_fail_count % 20 == 1) {
            printk("[BLE IMU] notify failed err=%d (fail_count=%u, ok_count=%u)\n",
                   err, notify_fail_count, notify_ok_count);
        }
    } else {
        notify_ok_count++;
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
        active_buf = 1 - active_buf;   /* swap immediately, don't wait on TX */
        sample_idx = 0;
        flush_batch(full_buf);
    }
}