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
#include <zmk/events/activity_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/activity.h>
#include <zmk/sensors.h>

LOG_MODULE_REGISTER(ble_toggle, CONFIG_LOG_DEFAULT_LEVEL);

/* ===================================================
 * WS2812 状态灯
 * =================================================== */

#define TOTAL_LEDS 1

static const struct device *led_strip_dev;
static struct led_rgb pixels[TOTAL_LEDS];

static bool led_blink_on = true;
static bool is_sleeping = false;

enum led_state {
    LED_STATE_USB,
    LED_STATE_BLE_PAIRING,
    LED_STATE_BLE_CONNECTED,
    LED_STATE_LOW_BATTERY,
    LED_STATE_OFF,
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
            set_led(0, 50, 0);
            break;
        case LED_STATE_BLE_PAIRING:
            if (led_blink_on) {
                set_led(0, 0, 50);
            } else {
                set_led(0, 0, 0);
            }
            led_blink_on = !led_blink_on;
            break;
        case LED_STATE_BLE_CONNECTED:
            set_led(0, 0, 50);
            break;
        case LED_STATE_LOW_BATTERY:
            if (led_blink_on) {
                set_led(50, 0, 0);
            } else {
                set_led(0, 0, 0);
            }
            led_blink_on = !led_blink_on;
            break;
        case LED_STATE_OFF:
        default:
            set_led(0, 0, 0);
            break;
    }
}

/* ===================================================
 * 状态检测主循环
 * =================================================== */

static void check_status(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(status_check_work, check_status);

static void check_status(struct k_work *work) {
    if (is_sleeping) {
        current_led_state = LED_STATE_OFF;
        update_status_led();
        return;
    }

    struct zmk_endpoint_instance endpoint = zmk_endpoint_get_selected();
    uint8_t battery_level = zmk_battery_state_of_charge();

    if (battery_level > 0 && battery_level < 10) {
        current_led_state = LED_STATE_LOW_BATTERY;
    } else if (endpoint.transport == ZMK_TRANSPORT_USB) {
        current_led_state = LED_STATE_USB;
    } else {
        if (zmk_ble_active_profile_is_connected()) {
            current_led_state = LED_STATE_BLE_CONNECTED;
        } else {
            current_led_state = LED_STATE_BLE_PAIRING;
        }
    }

    update_status_led();

    if (current_led_state == LED_STATE_BLE_PAIRING ||
        current_led_state == LED_STATE_LOW_BATTERY) {
        k_work_schedule(&status_check_work, K_MSEC(500));
    } else {
        k_work_schedule(&status_check_work, K_SECONDS(3));
    }
}

/* ===================================================
 * 休眠/唤醒 事件监听
 * =================================================== */

static int activity_event_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *activity_event =
        as_zmk_activity_state_changed(eh);
    if (activity_event == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    switch (activity_event->state) {
        case ZMK_ACTIVITY_ACTIVE:
            LOG_INF("Activity: ACTIVE - LED on");
            is_sleeping = false;
            k_work_schedule(&status_check_work, K_MSEC(100));
            break;

        case ZMK_ACTIVITY_IDLE:
            LOG_INF("Activity: IDLE - LED off");
            is_sleeping = true;
            k_work_cancel_delayable(&status_check_work);
            current_led_state = LED_STATE_OFF;
            update_status_led();
            break;

        case ZMK_ACTIVITY_SLEEP:
            LOG_INF("Activity: SLEEP - LED off");
            is_sleeping = true;
            k_work_cancel_delayable(&status_check_work);
            current_led_state = LED_STATE_OFF;
            update_status_led();
            break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(activity_led, activity_event_listener);
ZMK_SUBSCRIPTION(activity_led, zmk_activity_state_changed);

/* ===================================================
 * 旋钮转虚拟按键 (带软件去抖)
 * =================================================== */

#define ENCODER_CW_POSITION   6
#define ENCODER_CCW_POSITION  7
#define VIRTUAL_KEY_PRESS_MS  5

/* -------- 去抖参数（可根据手感微调）-------- *
 *
 *  ENCODER_DEBOUNCE_MS:
 *      同方向两次触发的最小间隔，低于此值的脉冲视为抖动丢弃。
 *      正常手速拧一格约 60-150ms，设 30ms 能滤掉绝大部分抖动。
 *      如果还有多触发 → 往上调（40、50）
 *      如果快速连拧丢步 → 往下调（20、15）
 *
 *  ENCODER_REVERSE_LOCKOUT_MS:
 *      方向反转后的锁定窗口。在 detent 边缘，触点可能
 *      短暂来回跳变产生假的"反转"，此窗口内的反转事件被丢弃。
 *      如果旋钮停着不动偶尔自己跳一下 → 往上调（120、150）
 */
#define ENCODER_DEBOUNCE_MS          30
#define ENCODER_REVERSE_LOCKOUT_MS   80

/* 去抖状态变量 */
static int64_t enc_last_trigger_time    = 0;   /* 上次成功触发的时间戳 */
static int     enc_last_direction       = 0;   /* +1=CW, -1=CCW, 0=初始 */
static int64_t enc_last_reverse_time    = 0;   /* 上次方向改变的时间戳 */

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
            LOG_INF("Sensor rotation raw: value=%d", value);

            if (value == 0) {
                return ZMK_EV_EVENT_HANDLED;
            }

            /* ---- 去抖逻辑开始 ---- */
            int direction = (value > 0) ? 1 : -1;
            int64_t now = k_uptime_get();

            /* 过滤层1：方向反转锁定
             * 如果距离上次方向改变时间太短，认为是 detent 边缘抖动 */
            if (enc_last_direction != 0 && direction != enc_last_direction) {
                if ((now - enc_last_reverse_time) < ENCODER_REVERSE_LOCKOUT_MS) {
                    LOG_DBG("Encoder debounce: reverse too fast, dropped");
                    return ZMK_EV_EVENT_HANDLED;
                }
                /* 真正的反转，记录时间并重置 */
                enc_last_reverse_time = now;
            }

            /* 过滤层2：同方向最小间隔去抖
             * 同一方向上两次触发间隔太短，认为是机械抖动 */
            if (direction == enc_last_direction) {
                if ((now - enc_last_trigger_time) < ENCODER_DEBOUNCE_MS) {
                    LOG_DBG("Encoder debounce: same-dir too fast (%lld ms), dropped",
                            now - enc_last_trigger_time);
                    return ZMK_EV_EVENT_HANDLED;
                }
            }

            /* ---- 去抖通过，记录状态并触发按键 ---- */
            enc_last_direction = direction;
            enc_last_trigger_time = now;

            LOG_INF("Encoder accepted: dir=%d", direction);

            if (direction > 0) {
                k_work_submit(&cw_work);
            } else {
                k_work_submit(&ccw_work);
            }
            /* ---- 去抖逻辑结束 ---- */

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

    LOG_INF("BLE Toggle + Encoder-to-Keys initialized (with debounce)");

    k_work_schedule(&status_check_work, K_SECONDS(5));
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
