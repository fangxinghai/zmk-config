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

/* ===================================================
 * 蓝牙配置颜色方案
 *
 * 5个蓝牙配置 (0-4) 用从深到浅的蓝色区分：
 *
 *   配置 0 → 最深蓝  (0,  0, 50)  纯蓝，无混色
 *   配置 1 → 深蓝    (5,  5, 50)  微微偏白
 *   配置 2 → 中蓝    (12, 12, 50) 明显偏浅
 *   配置 3 → 浅蓝    (22, 22, 50) 接近天蓝
 *   配置 4 → 最浅蓝  (35, 35, 50) 接近白蓝
 *
 * 越往后越浅（加白光降饱和度），肉眼可区分。
 * =================================================== */

#define BT_PROFILE_COUNT 5

static const struct led_rgb bt_colors[BT_PROFILE_COUNT] = {
    { .r = 0,  .g = 0,  .b = 50 },   /* 配置 0: 最深蓝 (纯蓝) */
    { .r = 5,  .g = 5,  .b = 50 },   /* 配置 1: 深蓝 */
    { .r = 12, .g = 12, .b = 50 },   /* 配置 2: 中蓝 */
    { .r = 22, .g = 22, .b = 50 },   /* 配置 3: 浅蓝 */
    { .r = 35, .g = 35, .b = 50 },   /* 配置 4: 最浅蓝 (白蓝) */
};

/* ===================================================
 * 状态灯控制
 * =================================================== */

static void set_led(uint8_t r, uint8_t g, uint8_t b) {
    if (led_strip_dev == NULL) return;
    pixels[0].r = r;
    pixels[0].g = g;
    pixels[0].b = b;
    led_strip_update_rgb(led_strip_dev, pixels, TOTAL_LEDS);
}

static void set_led_off(void) {
    set_led(0, 0, 0);
}

static void set_led_rgb(const struct led_rgb *c) {
    set_led(c->r, c->g, c->b);
}

/* ===================================================
 * 状态检测主循环
 *
 * 优先级 (高→低):
 *   1. 休眠        → 灭
 *   2. 低电量 <10% → 红色闪烁
 *   3. USB         → 绿色常亮
 *   4. 蓝牙配对中  → 对应配置的蓝色闪烁
 *   5. 蓝牙已连接  → 对应配置的蓝色常亮
 * =================================================== */

static void check_status(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(status_check_work, check_status);

static void check_status(struct k_work *work) {
    if (is_sleeping) {
        set_led_off();
        return;
    }

    struct zmk_endpoint_instance endpoint = zmk_endpoint_get_selected();
    uint8_t battery_level = zmk_battery_state_of_charge();
    bool low_batt = (battery_level > 0 && battery_level < 10);
    bool usb = (endpoint.transport == ZMK_TRANSPORT_USB);

    /* 获取当前蓝牙配置号对应的颜色 */
    int bt_profile = zmk_ble_active_profile_index();
    if (bt_profile < 0 || bt_profile >= BT_PROFILE_COUNT) {
        bt_profile = 0;
    }
    const struct led_rgb *bt_color = &bt_colors[bt_profile];

    /* 翻转闪烁相位 */
    led_blink_on = !led_blink_on;

    /* ---- 按优先级渲染 ---- */

    if (low_batt) {
        /* 低电量：红色闪烁 */
        if (led_blink_on) {
            set_led(50, 0, 0);
        } else {
            set_led_off();
        }
        k_work_schedule(&status_check_work, K_MSEC(500));

    } else if (usb) {
        /* USB：绿色常亮 */
        set_led(0, 50, 0);
        k_work_schedule(&status_check_work, K_SECONDS(3));

    } else if (zmk_ble_active_profile_is_connected()) {
        /* 蓝牙已连接：对应配置的蓝色常亮 */
        set_led_rgb(bt_color);
        k_work_schedule(&status_check_work, K_SECONDS(3));

    } else {
        /* 蓝牙配对中：对应配置的蓝色闪烁 */
        if (led_blink_on) {
            set_led_rgb(bt_color);
        } else {
            set_led_off();
        }
        k_work_schedule(&status_check_work, K_MSEC(500));
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
            set_led_off();
            break;

        case ZMK_ACTIVITY_SLEEP:
            LOG_INF("Activity: SLEEP - LED off");
            is_sleeping = true;
            k_work_cancel_delayable(&status_check_work);
            set_led_off();
            break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(activity_led, activity_event_listener);
ZMK_SUBSCRIPTION(activity_led, zmk_activity_state_changed);

/* ===================================================
 * 旋钮转虚拟按键（计数分频 + 反转保护）
 *
 * 两层防御：
 *
 * 1. 计数分频：
 *    EC11 每个物理 click 固定产生 2 次 event，
 *    每 2 次同方向 event 才触发 1 次按键。
 *
 * 2. 反转保护：
 *    慢拧时 detent 边缘可能产生 1 次假反向脉冲。
 *    方向反转后不立刻执行，要求连续收到
 *    REVERSE_CONFIRM_COUNT 次同方向 event 才确认反转。
 *    如果确认前又收到原方向 event，说明是假反转，
 *    丢弃之前的反向 event，恢复原方向计数。
 *
 * ENCODER_EVENTS_PER_CLICK:
 *    每格固定 2 次 event。换编码器可调。
 *
 * REVERSE_CONFIRM_COUNT:
 *    反转后需要连续多少次同方向 event 才确认。
 *    设 2 = 必须连续 2 次反向才承认反转（1 次反向丢弃）。
 *    这能过滤掉 detent 边缘的单次假反转脉冲。
 * =================================================== */

#define ENCODER_CW_POSITION         6
#define ENCODER_CCW_POSITION        7
#define VIRTUAL_KEY_PRESS_MS        5
#define ENCODER_EVENTS_PER_CLICK    2
#define REVERSE_CONFIRM_COUNT       2

static int  enc_confirmed_dir = 0;
static int  enc_pending_dir = 0;
static int  enc_pending_count = 0;
static int  enc_fire_counter = 0;

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

static void encoder_fire(int direction) {
    LOG_INF("Encoder fire: dir=%d", direction);
    if (direction > 0) {
        k_work_submit(&cw_work);
    } else {
        k_work_submit(&ccw_work);
    }
}

static int sensor_event_listener(const zmk_event_t *eh) {
    struct zmk_sensor_event *sensor_event = as_zmk_sensor_event(eh);
    if (sensor_event == NULL) return ZMK_EV_EVENT_BUBBLE;

    const struct zmk_sensor_channel_data *channel_data = sensor_event->channel_data;
    int channel_data_size = sensor_event->channel_data_size;

    for (int i = 0; i < channel_data_size; i++) {
        if (channel_data[i].channel == SENSOR_CHAN_ROTATION) {
            int value = sensor_value_to_micro(&channel_data[i].value);
            if (value == 0) return ZMK_EV_EVENT_HANDLED;

            int direction = (value > 0) ? 1 : -1;

            /* ---- 初始状态：第一次收到 event ---- */
            if (enc_confirmed_dir == 0) {
                enc_confirmed_dir = direction;
                enc_fire_counter = 1;
                enc_pending_dir = 0;
                enc_pending_count = 0;
                LOG_DBG("Encoder init: dir=%d", direction);
                if (enc_fire_counter >= ENCODER_EVENTS_PER_CLICK) {
                    enc_fire_counter = 0;
                    encoder_fire(direction);
                }
                return ZMK_EV_EVENT_HANDLED;
            }

            /* ---- 同方向：正常计数 ---- */
            if (direction == enc_confirmed_dir) {
                if (enc_pending_count > 0) {
                    LOG_DBG("Encoder: false reverse discarded (pending=%d)", enc_pending_count);
                    enc_pending_dir = 0;
                    enc_pending_count = 0;
                }

                enc_fire_counter++;

                LOG_DBG("Encoder: confirmed dir=%d, count=%d/%d",
                        direction, enc_fire_counter, ENCODER_EVENTS_PER_CLICK);

                if (enc_fire_counter >= ENCODER_EVENTS_PER_CLICK) {
                    enc_fire_counter = 0;
                    encoder_fire(direction);
                }
                return ZMK_EV_EVENT_HANDLED;
            }

            /* ---- 反方向：进入待确认流程 ---- */
            if (enc_pending_dir == direction) {
                enc_pending_count++;
            } else {
                enc_pending_dir = direction;
                enc_pending_count = 1;
            }

            LOG_DBG("Encoder: reverse pending dir=%d, count=%d/%d",
                    direction, enc_pending_count, REVERSE_CONFIRM_COUNT);

            if (enc_pending_count >= REVERSE_CONFIRM_COUNT) {
                LOG_INF("Encoder: reverse confirmed %d -> %d",
                        enc_confirmed_dir, enc_pending_dir);

                enc_confirmed_dir = enc_pending_dir;
                enc_fire_counter = enc_pending_count;
                enc_pending_dir = 0;
                enc_pending_count = 0;

                if (enc_fire_counter >= ENCODER_EVENTS_PER_CLICK) {
                    enc_fire_counter = 0;
                    encoder_fire(enc_confirmed_dir);
                }
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

    LOG_INF("BLE Toggle + Encoder-to-Keys initialized");

    k_work_schedule(&status_check_work, K_MSEC(500));
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
