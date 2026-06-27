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

/* NRF52840 GPREGRET 寄存器直接地址，零依赖 */
#define NRF_POWER_GPREGRET_REG  (*(volatile uint32_t *)0x40000051UL)

LOG_MODULE_REGISTER(ble_toggle, CONFIG_LOG_DEFAULT_LEVEL);

#define STATUS_LEDS 1

static const struct device *status_led_dev;
static struct led_rgb status_pixel[STATUS_LEDS];

static bool blink_phase = true;
static bool is_sleeping = false;

#define BT_PROFILE_COUNT 5

static const struct led_rgb bt_profile_colors[BT_PROFILE_COUNT] = {
    { .r = 0,  .g = 0,  .b = 50 },
    { .r = 0,  .g = 15, .b = 50 },
    { .r = 0,  .g = 30, .b = 50 },
    { .r = 15, .g = 0,  .b = 50 },
    { .r = 30, .g = 0,  .b = 50 },
};

static void status_led_set(uint8_t r, uint8_t g, uint8_t b) {
    if (status_led_dev == NULL) return;
    status_pixel[0].r = r;
    status_pixel[0].g = g;
    status_pixel[0].b = b;
    led_strip_update_rgb(status_led_dev, status_pixel, STATUS_LEDS);
}

static void status_led_off(void) {
    status_led_set(0, 0, 0);
}

static void status_led_set_rgb(const struct led_rgb *c) {
    status_led_set(c->r, c->g, c->b);
}

static void check_status(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(status_check_work, check_status);

static void check_status(struct k_work *work) {
    if (is_sleeping) {
        status_led_off();
        return;
    }

    struct zmk_endpoint_instance ep = zmk_endpoint_get_selected();
    uint8_t batt = zmk_battery_state_of_charge();
    bool low_batt = (batt > 0 && batt < 10);
    bool usb = (ep.transport == ZMK_TRANSPORT_USB);

    int bt_profile = zmk_ble_active_profile_index();
    if (bt_profile < 0 || bt_profile >= BT_PROFILE_COUNT) {
        bt_profile = 0;
    }
    const struct led_rgb *bt_color = &bt_profile_colors[bt_profile];

    blink_phase = !blink_phase;

    if (low_batt) {
        if (blink_phase) {
            status_led_set(50, 0, 0);
        } else {
            status_led_off();
        }
        k_work_schedule(&status_check_work, K_MSEC(500));

    } else if (usb) {
        status_led_set(0, 50, 0);
        k_work_schedule(&status_check_work, K_SECONDS(3));

    } else if (zmk_ble_active_profile_is_connected()) {
        status_led_set_rgb(bt_color);
        k_work_schedule(&status_check_work, K_SECONDS(3));

    } else {
        if (blink_phase) {
            status_led_set_rgb(bt_color);
        } else {
            status_led_off();
        }
        k_work_schedule(&status_check_work, K_MSEC(500));
    }
}

static int activity_event_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev == NULL) return ZMK_EV_EVENT_BUBBLE;

    switch (ev->state) {
        case ZMK_ACTIVITY_ACTIVE:
            LOG_INF("Activity: ACTIVE");
            is_sleeping = false;
            k_work_schedule(&status_check_work, K_MSEC(100));
            break;

        case ZMK_ACTIVITY_IDLE:
        case ZMK_ACTIVITY_SLEEP:
            LOG_INF("Activity: IDLE/SLEEP - status LED off");
            is_sleeping = true;
            k_work_cancel_delayable(&status_check_work);
            status_led_off();
            break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(activity_led, activity_event_listener);
ZMK_SUBSCRIPTION(activity_led, zmk_activity_state_changed);

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
            struct sensor_value val_copy = channel_data[i].value;
            int value = sensor_value_to_micro(&val_copy);
            if (value == 0) return ZMK_EV_EVENT_HANDLED;

            int direction = (value > 0) ? 1 : -1;

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

static int ble_toggle_init(void) {
    NRF_POWER_GPREGRET_REG = 0x00UL;

    status_led_dev = DEVICE_DT_GET(DT_NODELABEL(status_led));
    if (!device_is_ready(status_led_dev)) {
        LOG_WRN("Status LED (P0.29) not ready");
        status_led_dev = NULL;
    }
    memset(status_pixel, 0, sizeof(status_pixel));

        k_work_init(&cw_work, cw_work_handler);
    k_work_init(&ccw_work, ccw_work_handler);

    LOG_INF("Status LED + Encoder initialized");
    LOG_INF("GPREGRET cleared");
    LOG_INF("Backlight managed by ZMK RGB underglow");

    k_work_schedule(&status_check_work, K_SECONDS(3));
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
