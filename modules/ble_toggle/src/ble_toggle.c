#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <string.h>
#include <zmk/ble.h>
#include <zmk/usb.h>
#include <zmk/endpoints_types.h>
#include <zmk/endpoints.h>
#include <zmk/battery.h>

/* 只有 1 颗 WS2812 */
#define TOTAL_LEDS 1

/* LED strip 设备 */
static const struct device *led_strip_dev;
static struct led_rgb pixels[TOTAL_LEDS];

/* 蓝牙状态 */
static bool ble_forced_off = false;
static bool user_selected_ble = false;

/* LED 闪烁控制 */
static bool led_blink_on = true;

/* LED 状态 */
enum led_state {
    LED_STATE_USB,
    LED_STATE_BLE_PAIRING,
    LED_STATE_BLE_CONNECTED,
    LED_STATE_LOW_BATTERY,
};

static enum led_state current_led_state = LED_STATE_USB;

/* 设置 LED 颜色 */
static void set_led(uint8_t r, uint8_t g, uint8_t b) {
    if (led_strip_dev == NULL) {
        return;
    }
    pixels[0].r = r;
    pixels[0].g = g;
    pixels[0].b = b;
    led_strip_update_rgb(led_strip_dev, pixels, TOTAL_LEDS);
}

/* 更新状态灯 */
static void update_status_led(void) {
    switch (current_led_state) {
        case LED_STATE_USB:
            /* 绿色常亮 */
            set_led(0, 50, 0);
            break;

        case LED_STATE_BLE_PAIRING:
            /* 蓝色快闪 */
            if (led_blink_on) {
                set_led(0, 0, 50);
            } else {
                set_led(0, 0, 0);
            }
            led_blink_on = !led_blink_on;
            break;

        case LED_STATE_BLE_CONNECTED:
            /* 蓝色常亮 */
            set_led(0, 0, 50);
            break;

        case LED_STATE_LOW_BATTERY:
            /* 红色闪烁 */
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

/* 蓝牙控制 */
static void disable_ble(void) {
    if (!ble_forced_off) {
        bt_le_adv_stop();
        ble_forced_off = true;
    }
}

static void enable_ble(void) {
    if (ble_forced_off) {
        ble_forced_off = false;
        int err = bt_le_adv_start(
            BT_LE_ADV_CONN_ONE_TIME,
            NULL, 0, NULL, 0);
        if (err) {
            zmk_ble_prof_select(0);
        }
    }
}

/* 主检测循环 */
static void check_status(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(status_check_work, check_status);

static void check_status(struct k_work *work) {
    struct zmk_endpoint_instance endpoint = zmk_endpoint_get_selected();

    /* 蓝牙开关控制 */
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

    /* 电池电量检测 */
    uint8_t battery_level = zmk_battery_state_of_charge();

    /* 确定 LED 状态 */
    if (battery_level > 0 && battery_level < 10) {
        /* 电量低于 10% 最高优先级 */
        current_led_state = LED_STATE_LOW_BATTERY;
    } else if (endpoint.transport == ZMK_TRANSPORT_USB && !user_selected_ble) {
        /* USB 模式 */
        current_led_state = LED_STATE_USB;
    } else if (user_selected_ble || endpoint.transport == ZMK_TRANSPORT_BLE) {
        /* 蓝牙模式 */
        if (zmk_ble_active_profile_is_connected()) {
            current_led_state = LED_STATE_BLE_CONNECTED;
        } else {
            current_led_state = LED_STATE_BLE_PAIRING;
        }
    }

    /* 更新 LED */
    update_status_led();

    /* 闪烁状态 200ms，常亮状态 2 秒 */
    if (current_led_state == LED_STATE_BLE_PAIRING ||
        current_led_state == LED_STATE_LOW_BATTERY) {
        k_work_schedule(&status_check_work, K_MSEC(200));
    } else {
        k_work_schedule(&status_check_work, K_SECONDS(2));
    }
}

static int ble_toggle_init(void) {
    led_strip_dev = DEVICE_DT_GET(DT_NODELABEL(led_strip));
    if (!device_is_ready(led_strip_dev)) {
        led_strip_dev = NULL;
    }

    memset(pixels, 0, sizeof(pixels));

    k_work_schedule(&status_check_work, K_SECONDS(5));
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
