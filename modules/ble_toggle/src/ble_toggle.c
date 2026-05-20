#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zmk/ble.h>
#include <zmk/usb.h>
#include <zmk/endpoints_types.h>
#include <zmk/endpoints.h>
#include <zmk/battery.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/event_manager.h>
#include <zmk/sensors.h>

LOG_MODULE_REGISTER(ble_toggle, CONFIG_LOG_DEFAULT_LEVEL);

/* ===================================================
 * WS2812 状态灯
 * =================================================== */

#define TOTAL_LEDS 1

static const struct device *led_strip_dev;
static struct led_rgb pixels[TOTAL_LEDS];

static bool led_blink_on = true;

enum led_state {
    LED_STATE_USB,
    LED_STATE_BLE_PAIRING,
    LED_STATE_BLE_CONNECTED,
    LED_STATE_LOW_BATTERY,
};

static enum led_state current_led_state = LED_STATE_USB;

static void set_led(uint8_t r, uint8_t g, uint8_t b) {
    if (led_strip_dev == NULL) {
        return;
    }
    pixels[0].r = r;
    pixels[0].g = g;
    pixels[0].b = b;
    led_strip_update_rgb(led_strip_dev, pixels, TOTAL_LEDS);
}

static void update_status_led(void) {
    switch (current_led_state) {
        case LED_STATE_USB:
            set_led(0, 50, 0);     /* 绿色常亮 */
            break;

        case LED_STATE_BLE_PAIRING:
            if (led_blink_on) {
                set_led(0, 0, 50);  /* 蓝色闪烁 */
            } else {
                set_led(0, 0, 0);
            }
            led_blink_on = !led_blink_on;
            break;

        case LED_STATE_BLE_CONNECTED:
            set_led(0, 0, 50);     /* 蓝色常亮 */
            break;

        case LED_STATE_LOW_BATTERY:
            if (led_blink_on) {
                set_led(50, 0, 0);  /* 红色闪烁 */
            } else {
                set_led(0, 0, 0);
            }
            led_blink_on = !led_blink_on;
            break;

        default:
            set_led(0, 0, 0);
            break;
    }
}

/* ===================================================
 * 状态检测主循环（纯状态灯，不干预蓝牙）
 *
 * 蓝牙的开关完全交给 ZMK 内部管理：
 *   - USB 插入时 ZMK 自动走 USB
 *   - USB 拔出时 ZMK 自动切到 BLE 并广播
 *   - 用户通过 OUT_BLE / OUT_USB 手动切换
 *
 * 我们只读取状态来控制 LED 颜色，不做任何
 * bt_le_adv_stop / bt_le_adv_start 操作。
 * =================================================== */

static void check_status(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(status_check_work, check_status);

static void check_status(struct k_work *work) {
    struct zmk_endpoint_instance endpoint = zmk_endpoint_get_selected();
    uint8_t battery_level = zmk_battery_state_of_charge();

    /* 判断 LED 状态（优先级：低电量 > USB/BLE） */
    if (battery_level > 0 && battery_level < 10) {
        current_led_state = LED_STATE_LOW_BATTERY;
    } else if (endpoint.transport == ZMK_TRANSPORT_USB) {
        current_led_state = LED_STATE_USB;
    } else {
        /* BLE 模式 */
        if (zmk_ble_active_profile_is_connected()) {
            current_led_state = LED_STATE_BLE_CONNECTED;
        } else {
            current_led_state = LED_STATE_BLE_PAIRING;
        }
    }

    update_status_led();

    /* 闪烁状态刷新更快 */
    if (current_led_state == LED_STATE_BLE_PAIRING ||
        current_led_state == LED_STATE_LOW_BATTERY) {
        k_work_schedule(&status_check_work, K_MSEC(500));
    } else {
        k_work_schedule(&status_check_work, K_SECONDS(3));
    }
}

/* ===================================================
 * 旋钮转虚拟按键
 *
 * 拦截 ZMK sensor event，转换为虚拟按键
 * 顺时针转一格 → position 6 按下+释放一次
 * 逆时针转一格 → position 7 按下+释放一次
 * =================================================== */

#define ENCODER_CW_POSITION   6
#define ENCODER_CCW_POSITION  7
#define VIRTUAL_KEY_PRESS_MS  5

static void encoder_virtual_press(uint32_t position) {
    LOG_INF("Encoder virtual press: position=%d", position);

    raise_zmk_position_state_changed((struct zmk_position_state_changed){
        .source = 0,
        .position = position,
        .state = true,
        .timestamp = k_uptime_get(),
    });

    k_msleep(VIRTUAL_KEY_PRESS_MS);

    raise_zmk_position_state_changed((struct zmk_position_state_changed){
        .source = 0,
        .position = position,
        .state = false,
        .timestamp = k_uptime_get(),
    });
}

static struct k_work cw_work;
static struct k_work ccw_work;

static void cw_work_handler(struct k_work *work) {
    encoder_virtual_press(ENCODER_CW_POSITION);
}

static void ccw_work_handler(struct k_work *work) {
    encoder_virtual_press(ENCODER_CCW_POSITION);
}

/*
 * ZMK sensor event listener
 * 拦截 EC11 产生的 sensor event，判断方向，触发虚拟按键
 * 返回 ZMK_EV_EVENT_HANDLED 阻止 ZMK 继续处理
 */
static int sensor_event_listener(const zmk_event_t *eh) {
    struct zmk_sensor_event *sensor_event = as_zmk_sensor_event(eh);
    if (sensor_event == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_sensor_channel_data *channel_data = sensor_event->channel_data;
    int channel_data_size = sensor_event->channel_data_size;

    for (int i = 0; i < channel_data_size; i++) {
        if (channel_data[i].channel == SENSOR_CHAN_ROTATION) {
            int value = sensor_value_to_micro(&channel_data[i].value);
            LOG_INF("Sensor rotation: value=%d", value);

            if (value > 0) {
                k_work_submit(&cw_work);
            } else if (value < 0) {
                k_work_submit(&ccw_work);
            }
            return ZMK_EV_EVENT_HANDLED;
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(encoder_to_keys, sensor_event_listener);
ZMK_SUBSCRIPTION(encoder_to_keys, zmk_sensor_event);

/* ===================================================
 * 初始化
 * =================================================== */

static int ble_toggle_init(void) {
    led_strip_dev = DEVICE_DT_GET(DT_NODELABEL(led_strip));
    if (!device_is_ready(led_strip_dev)) {
        LOG_WRN("LED strip not ready");
        led_strip_dev = NULL;
    }
    memset(pixels, 0, sizeof(pixels));

    k_work_init(&cw_work, cw_work_handler);
    k_work_init(&ccw_work, ccw_work_handler);

    LOG_INF("BLE Toggle initialized (LED status only, no BLE control)");
    LOG_INF("Encoder-to-Keys: CW=pos%d, CCW=pos%d",
            ENCODER_CW_POSITION, ENCODER_CCW_POSITION);

    k_work_schedule(&status_check_work, K_SECONDS(5));
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
