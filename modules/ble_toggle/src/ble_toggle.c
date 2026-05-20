#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <string.h>
#include <zmk/ble.h>
#include <zmk/usb.h>
#include <zmk/endpoints_types.h>
#include <zmk/endpoints.h>
#include <zmk/battery.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/event_manager.h>

/* ===================================================
 * WS2812 状态灯
 * =================================================== */

#define TOTAL_LEDS 1

static const struct device *led_strip_dev;
static struct led_rgb pixels[TOTAL_LEDS];

static bool ble_forced_off = false;
static bool user_selected_ble = false;
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

        default:
            set_led(0, 0, 0);
            break;
    }
}

/* ===================================================
 * 蓝牙控制
 * =================================================== */

static void disable_ble(void) {
    if (!ble_forced_off) {
        bt_le_adv_stop();
        ble_forced_off = true;
    }
}

static void enable_ble(void) {
    if (ble_forced_off) {
        ble_forced_off = false;
        /* 直接让 ZMK 管理广播恢复 */
        zmk_ble_prof_select(0);
    }
}

/* ===================================================
 * 状态检测主循环
 * =================================================== */

static void check_status(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(status_check_work, check_status);

static void check_status(struct k_work *work) {
    struct zmk_endpoint_instance endpoint = zmk_endpoint_get_selected();

    if (endpoint.transport == ZMK_TRANSPORT_USB) {
        if (!user_selected_ble) {
            disable_ble();
        }
    } else {
        user_selected_ble = true;
        enable_ble();
    }

    if (endpoint.transport == ZMK_TRANSPORT_USB) {
        user_selected_ble = false;
    }

    uint8_t battery_level = zmk_battery_state_of_charge();

    if (battery_level > 0 && battery_level < 10) {
        current_led_state = LED_STATE_LOW_BATTERY;
    } else if (endpoint.transport == ZMK_TRANSPORT_USB && !user_selected_ble) {
        current_led_state = LED_STATE_USB;
    } else if (user_selected_ble || endpoint.transport == ZMK_TRANSPORT_BLE) {
        if (zmk_ble_active_profile_is_connected()) {
            current_led_state = LED_STATE_BLE_CONNECTED;
        } else {
            current_led_state = LED_STATE_BLE_PAIRING;
        }
    }

    update_status_led();

    if (current_led_state == LED_STATE_BLE_PAIRING ||
        current_led_state == LED_STATE_LOW_BATTERY) {
        k_work_schedule(&status_check_work, K_MSEC(200));
    } else {
        k_work_schedule(&status_check_work, K_SECONDS(2));
    }
}

/* ===================================================
 * 旋钮转虚拟按键
 *
 * 顺时针转一格 → position 6 按下+释放一次
 * 逆时针转一格 → position 7 按下+释放一次
 * =================================================== */

#define ENCODER_CW_POSITION   6
#define ENCODER_CCW_POSITION  7
#define VIRTUAL_KEY_PRESS_MS  5

static void encoder_virtual_press(uint32_t position) {
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

/* Zephyr 4.x INPUT_CALLBACK_DEFINE 需要3个参数：device, callback, user_data */
static void encoder_input_cb(struct input_event *evt, void *user_data) {
    if (evt->code != INPUT_REL_WHEEL) {
        return;
    }

    if (evt->value > 0) {
        k_work_submit(&cw_work);
    } else if (evt->value < 0) {
        k_work_submit(&ccw_work);
    }
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(encoder)), encoder_input_cb, NULL);

/* ===================================================
 * 初始化
 * =================================================== */

static int ble_toggle_init(void) {
    /* LED 初始化 */
    led_strip_dev = DEVICE_DT_GET(DT_NODELABEL(led_strip));
    if (!device_is_ready(led_strip_dev)) {
        led_strip_dev = NULL;
    }
    memset(pixels, 0, sizeof(pixels));

    /* 旋钮工作队列初始化 */
    k_work_init(&cw_work, cw_work_handler);
    k_work_init(&ccw_work, ccw_work_handler);

    /* 启动状态检测 */
    k_work_schedule(&status_check_work, K_SECONDS(5));
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
