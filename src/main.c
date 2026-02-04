/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define BT_UUID_ZEPHYR_SERVER_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_ZEPHYR_READ_VAL \
    BT_UUID_128_ENCODE(0xabcdef01, 0x2345, 0x6789, 0xabcd, 0xef0123456789)

#define BT_UUID_ZEPHYR_WRITE_VAL \
    BT_UUID_128_ENCODE(0xfedcba98, 0x7654, 0x3210, 0xfedc, 0xba9876543210)

#define BT_UUID_ZEPHYR_NOTIFY_VAL \
    BT_UUID_128_ENCODE(0x11111111, 0x2222, 0x3333, 0x4444, 0x555555555555)

static struct bt_uuid_128 zephyr_server_uuid = BT_UUID_INIT_128(BT_UUID_ZEPHYR_SERVER_VAL);
static struct bt_uuid_128 zephyr_write_uuid = BT_UUID_INIT_128(BT_UUID_ZEPHYR_WRITE_VAL);
static struct bt_uuid_128 zephyr_read_uuid = BT_UUID_INIT_128(BT_UUID_ZEPHYR_READ_VAL);
static struct bt_uuid_128 zephyr_nortify_uuid = BT_UUID_INIT_128(BT_UUID_ZEPHYR_NOTIFY_VAL);

static uint8_t write_char_value[] = {0x00, 0x00, 0x00, 0x00};
static uint8_t read_char_value[] = {0x00, 0x00, 0x00, 0x00};
static uint8_t nortify_char_value[] = {0x00};
static uint8_t cccd_value[] = {0x00, 0x00};

ssize_t	read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	const uint8_t *value = attr->user_data;
    
    	LOG_INF("read characristic: %02x %02x %02x %02x\n", 
           value[0], value[1], value[2], value[3]);
    
    	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 
                            sizeof(read_char_value));
}

ssize_t	write_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flag) {
	uint8_t *value = attr->user_data;
    
	if (offset + len > sizeof(write_char_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
    
	memcpy(value + offset, buf, len);
	
	LOG_INF("write characristic: ");
	for (int i = 0; i < len; i++) {
		LOG_INF("%02x ", ((uint8_t *)buf)[i]);
	}
	
	return len;
}

// 添加一个简单的读回调用于写特征（虽然它是只写的，但需要一个读回调占位）
ssize_t write_read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
                           void *buf, uint16_t len, uint16_t offset) {
    const uint8_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 
                             sizeof(write_char_value));
}

/* CCCD 写的回调函数 */
static void cccd_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    
    LOG_INF("CCCD change: %s\n", notify_enabled ? "ennable notify" : "disable notify");
}

static struct bt_gatt_attr service_attrs[] = {
	BT_GATT_PRIMARY_SERVICE(&zephyr_server_uuid), // 声明主服务

	BT_GATT_CHARACTERISTIC(&zephyr_read_uuid.uuid, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_callback, NULL, read_char_value), // 声明可读特征
	BT_GATT_CHARACTERISTIC(&zephyr_write_uuid.uuid, BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, 
        write_read_callback, write_callback, write_char_value), // 声明可写特征
    BT_GATT_CHARACTERISTIC(&zephyr_nortify_uuid.uuid, BT_GATT_CHRC_NOTIFY, NULL, NULL, NULL, nortify_char_value),
    BT_GATT_CCC(cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), // 声明cccd
};

static struct bt_gatt_service service = BT_GATT_SERVICE(service_attrs); // 服务定义

static void define_service_init(void)
{
    /* 注册服务 */
    int err = bt_gatt_service_register(&service);
    if (err) {
        LOG_ERR("define service register error: %d\n", err);
    } else {
        LOG_INF("define service register successful\n");
    }
}

void send_custom_notification(uint8_t *data, uint16_t len)
{
    if (len == 0 || data == NULL) {
        return;
    }
    
    struct bt_gatt_notify_params params = {
        .attr = &service_attrs[6],
        .data = data,
        .len = len,
        .func = NULL,  /* 可选：通知发送完成的回调 */
    };
    
    /* 发送给所有连接的设备 */
    bt_gatt_notify(NULL, params.attr, data, len);
    
    /* 或者发送给特定连接 */
    // bt_gatt_notify(conn, &params);
}

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
#define RX_BUF_LEN      256
#define UART_RX_READY_MS    10000 // 10秒超时（或者设短一点，比如50ms，以此触发不定长接收）

static K_SEM_DEFINE(ble_init, 0, 1);
static K_MUTEX_DEFINE(is_conn);
static K_FIFO_DEFINE(uart_rx_fifo);

typedef struct {
    void *fifo_reserved; // Zephyr FIFO 需要这个保留字段在最前面
    uint8_t data[RX_BUF_LEN];
    size_t len;          // 使用 size_t 更标准
} uart_data_t;

static uint16_t current_mtu;
static struct bt_conn *current_conn;

static const struct device* uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

static void notif_enabled(bool enabled, void *ctx)
{
    ARG_UNUSED(ctx);
    LOG_INF("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);
    LOG_INF("%s() - Len: %d, Message: %.*s\n", __func__, len, len, (char *)data);
}

struct bt_nus_cb nus_listener = {
    .notif_enabled = notif_enabled,
    .received = received,
};

void mtu_update(struct bt_conn *conn, uint16_t tx, uint16_t rx) {
    LOG_INF("update mtu: TX: %d, RX: %d", tx, rx);
    current_mtu = tx > rx ? rx : tx;
}

static struct bt_gatt_cb gatt_callback = {
    .att_mtu_updated = mtu_update, //注册mtu更新回调函数
};

static void exchange_function(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params) {
    if (!err) {
		LOG_INF("MTU exchange successful");
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err) {
    LOG_INF("extern device is connected");

    current_conn = bt_conn_ref(conn);

    static struct bt_gatt_exchange_params exchange_param;
    exchange_param.func = exchange_function;

    err = bt_gatt_exchange_mtu(conn, &exchange_param);
    if (err) {
        LOG_ERR("MTU exchange request failed (err %d)", err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("extern device is disconnected");

    if (current_conn) {
        bt_conn_ref(current_conn);
        current_conn = NULL;
    }

    int err;
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("restart adv failed (err %d)", err);
    }
}

BT_CONN_CB_DEFINE(conn_callback) = {
    .connected = connected,
    .disconnected = disconnected,
};

void uart_callback_fun(const struct device *dev, struct uart_event *evt, void *user_data) {
    uart_data_t *buf;

    switch (evt->type) {
        case UART_RX_RDY: {
            buf = CONTAINER_OF(evt->data.rx.buf, uart_data_t, data[0]);
            buf->len += evt->data.rx.len;
            
            if (buf->len > 0) {
                uint8_t last_char = buf->data[buf->len - 1];
                if (last_char == '\n' || last_char == '\r') {
                    uart_rx_disable(uart_dev);
                }
            }
            break;
        }

        case UART_RX_DISABLED: {
            buf = k_calloc(1, sizeof(uart_data_t));
            if (buf) {
                buf->len = 0;
                uart_rx_enable(uart_dev, buf->data, RX_BUF_LEN, UART_RX_READY_MS);
            } else {
                LOG_ERR("UART OOM");
            }
            break;
        }

        case UART_RX_BUF_REQUEST: {
            buf = k_calloc(1, sizeof(uart_data_t));
            if (buf) {
                buf->len = 0;
                uart_rx_buf_rsp(uart_dev, buf->data, RX_BUF_LEN);
            }
            break;
        }

        case UART_RX_BUF_RELEASED: {
            buf = CONTAINER_OF(evt->data.rx_buf.buf, uart_data_t, data[0]);
            
            if (buf->len > 0) {
                k_fifo_put(&uart_rx_fifo, buf);
            } else {
                k_free(buf);
            }
            break;
        }

        default: break;
    }
}

static int uart_init(void) {
    int err;
    uart_data_t *rx;

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("Failed to device: %s not ready", uart_dev->name);
        return -ENODEV;
    }

    err = uart_callback_set(uart_dev, uart_callback_fun, NULL);
    if (err) {
        LOG_ERR("Failed callback set");
        return err;
    }

    rx = k_calloc(1, sizeof(uart_data_t));
    if (!rx) {
        return -ENOMEM;
    }
    rx->len = 0;

    err = uart_rx_enable(uart_dev, rx->data, RX_BUF_LEN, UART_RX_READY_MS);
    if (err) {
        LOG_ERR("Failed rx enable: %d", err);
        k_free(rx);
        return err;
    }

    return 0;
}

int main(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Failed to enable bluetooth: %d\n", err);
        return err;
    }

    define_service_init();

    err = bt_nus_cb_register(&nus_listener, NULL);
    if (err) return err;

    bt_gatt_cb_register(&gatt_callback);

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) return err;

    err = uart_init();
    if (err) {
        LOG_ERR("UART init failed: %d", err);
        return err;
    }

    k_sem_give(&ble_init);
    LOG_INF("Initialization complete\n");

    return 0;
}

void nus_send_thread(void *p1, void *p2, void *p3) {
    k_sem_take(&ble_init, K_FOREVER);
    k_sem_give(&ble_init);
    
    uart_data_t *buf; 
    
    while (1) {
        buf = k_fifo_get(&uart_rx_fifo, K_FOREVER);
        
        LOG_INF("BLE Send: %.*s", buf->len, buf->data);

        int err = bt_nus_send(NULL, buf->data, buf->len);
        if (err) {
            LOG_WRN("BLE send failed: %d (Not connected?)", err);
        }

        k_free(buf);
    }
}

void notify_send_thread(void *p1, void *p2, void *p3) {
    k_sem_take(&ble_init, K_FOREVER);
    k_sem_give(&ble_init);

    while (1) {
        if (current_conn != NULL) {
            bt_gatt_notify(current_conn, &service_attrs[6], nortify_char_value, sizeof(nortify_char_value));
            
            nortify_char_value[0]++;
        }

        k_sleep(K_MSEC(1000));
    }
}

void app_main(void)
{
    
}

K_THREAD_DEFINE(nus_thread, 4096, nus_send_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(notify_thread, 2048, notify_send_thread, NULL, NULL, NULL, 5, 0, 0);