#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>

#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/activity.h>
#include <zmk/sensors.h>

#include <dt-bindings/zmk/keys.h>

LOG_MODULE_REGISTER(ble_toggle, CONFIG_LOG_DEFAULT_LEVEL);

static bool is_sleeping = false;

/* =========================================================
 *  活动/休眠事件（用来暂停 ADC 采样省电）
 * ========================================================= */
static void volume_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(volume_work, volume_work_handler);

static int activity_event_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev == NULL) return ZMK_EV_EVENT_BUBBLE;

    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        is_sleeping = false;
        k_work_schedule(&volume_work, K_MSEC(100));
        break;
    case ZMK_ACTIVITY_IDLE:
    case ZMK_ACTIVITY_SLEEP:
        is_sleeping = true;
        k_work_cancel_delayable(&volume_work);
        break;
    }
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(activity_led, activity_event_listener);
ZMK_SUBSCRIPTION(activity_led, zmk_activity_state_changed);

/* =========================================================
 *  编码器：计数分频 + 反转保护
 * ========================================================= */
#define ENCODER_CW_POSITION       6
#define ENCODER_CCW_POSITION      7
#define VIRTUAL_KEY_PRESS_MS      5
#define ENCODER_EVENTS_PER_CLICK  2
#define REVERSE_CONFIRM_COUNT     2

static int enc_confirmed_dir = 0;
static int enc_pending_dir   = 0;
static int enc_pending_count = 0;
static int enc_fire_counter  = 0;

static void encoder_virtual_press(uint32_t position) {
    raise_zmk_position_state_changed((struct zmk_position_state_changed){
        .source = 0, .position = position, .state = true,
        .timestamp = k_uptime_get(),
    });
    k_msleep(VIRTUAL_KEY_PRESS_MS);
    raise_zmk_position_state_changed((struct zmk_position_state_changed){
        .source = 0, .position = position, .state = false,
        .timestamp = k_uptime_get(),
    });
}

static struct k_work cw_work;
static struct k_work ccw_work;
static void cw_work_handler(struct k_work *work)  { encoder_virtual_press(ENCODER_CW_POSITION); }
static void ccw_work_handler(struct k_work *work) { encoder_virtual_press(ENCODER_CCW_POSITION); }
static void encoder_fire(int direction) {
    if (direction > 0) k_work_submit(&cw_work);
    else               k_work_submit(&ccw_work);
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

            if (enc_confirmed_dir == 0) {
                enc_confirmed_dir = direction;
                enc_fire_counter  = 1;
                enc_pending_dir   = 0;
                enc_pending_count = 0;
                if (enc_fire_counter >= ENCODER_EVENTS_PER_CLICK) {
                    enc_fire_counter = 0;
                    encoder_fire(direction);
                }
                return ZMK_EV_EVENT_HANDLED;
            }
            if (direction == enc_confirmed_dir) {
                if (enc_pending_count > 0) { enc_pending_dir = 0; enc_pending_count = 0; }
                enc_fire_counter++;
                if (enc_fire_counter >= ENCODER_EVENTS_PER_CLICK) {
                    enc_fire_counter = 0;
                    encoder_fire(direction);
                }
                return ZMK_EV_EVENT_HANDLED;
            }
            if (enc_pending_dir == direction) enc_pending_count++;
            else { enc_pending_dir = direction; enc_pending_count = 1; }

            if (enc_pending_count >= REVERSE_CONFIRM_COUNT) {
                enc_confirmed_dir = enc_pending_dir;
                enc_fire_counter  = enc_pending_count;
                enc_pending_dir   = 0;
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

/* =========================================================
 *  ★ 滑动变阻器 → 音量 (P0.29 = AIN5)
 *
 *  改用虚拟按键路径：
 *    音量+ → position VOL_UP_POSITION   (键位 8)
 *    音量- → position VOL_DN_POSITION   (键位 9)
 *  → 在 keymap 里把 8/9 绑成 &kp C_VOL_UP / &kp C_VOL_DN
 * ========================================================= */
#define SAMPLE_INTERVAL_MS      50
#define FILTER_WIN              8
#define VOL_STEPS               50
#define VOL_DEADBAND            1
#define VOL_KEY_GAP_MS          15
#define VOL_MAX_STEP_PER_TICK   3

/* ★ 新增两个虚拟按键位置 (接在编码器 6/7 后面) */
#define VOL_UP_POSITION         8
#define VOL_DN_POSITION         9

#define ADC_NODE  DT_PATH(zephyr_user)
static const struct adc_dt_spec adc_chan =
    ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0);

static int16_t adc_raw_buf;
static struct adc_sequence adc_seq = {
    .buffer      = &adc_raw_buf,
    .buffer_size = sizeof(adc_raw_buf),
};

static int   filt_ring[FILTER_WIN];
static int   filt_idx  = 0;
static bool  filt_full = false;
static int   last_stable_vol = 0;
static bool  vol_initialized = false;

/* ★ 新版：走虚拟按键，不再调 zmk_endpoints_send_report */
static void send_vol_key(bool up) {
    uint32_t pos = up ? VOL_UP_POSITION : VOL_DN_POSITION;
    raise_zmk_position_state_changed((struct zmk_position_state_changed){
        .source = 0, .position = pos, .state = true,
        .timestamp = k_uptime_get(),
    });
    k_msleep(VIRTUAL_KEY_PRESS_MS);
    raise_zmk_position_state_changed((struct zmk_position_state_changed){
        .source = 0, .position = pos, .state = false,
        .timestamp = k_uptime_get(),
    });
}

static int adc_read_once(void) {
    int err = adc_read(adc_chan.dev, &adc_seq);
    if (err) {
        LOG_WRN("ADC read failed: %d", err);
        return -1;
    }
    int32_t val = adc_raw_buf;
    if (val < 0) val = 0;
    return val;
}

static int raw_to_vol(int raw) {
    const int RAW_MAX = (1 << 12) - 1;
    if (raw < 0) raw = 0;
    if (raw > RAW_MAX) raw = RAW_MAX;
    return (raw * VOL_STEPS + RAW_MAX / 2) / RAW_MAX;
}

static void volume_work_handler(struct k_work *work) {
    if (is_sleeping) {
        k_work_schedule(&volume_work, K_SECONDS(1));
        return;
    }

    int raw = adc_read_once();
    if (raw >= 0) {
        filt_ring[filt_idx] = raw;
        filt_idx = (filt_idx + 1) % FILTER_WIN;
        if (filt_idx == 0) filt_full = true;

        int cnt = filt_full ? FILTER_WIN : filt_idx;
        if (cnt == 0) cnt = FILTER_WIN;
        int sum = 0;
        for (int i = 0; i < cnt; i++) sum += filt_ring[i];
        int avg = sum / cnt;

        int target_vol = raw_to_vol(avg);

        if (!vol_initialized) {
            if (filt_full) {
                last_stable_vol = target_vol;
                vol_initialized = true;
                LOG_INF("Volume slider init at step %d (raw=%d)", target_vol, avg);
            }
        } else {
            int diff = target_vol - last_stable_vol;
            if (abs(diff) > VOL_DEADBAND) {
                int steps = abs(diff);
                if (steps > VOL_MAX_STEP_PER_TICK) steps = VOL_MAX_STEP_PER_TICK;
                bool up = (diff > 0);

                LOG_DBG("Vol %d -> %d, sending %d %s",
                        last_stable_vol, target_vol, steps, up ? "UP" : "DOWN");

                for (int i = 0; i < steps; i++) {
                    send_vol_key(up);
                    k_msleep(VOL_KEY_GAP_MS);
                }
                last_stable_vol += up ? steps : -steps;
            }
        }
    }

    k_work_schedule(&volume_work, K_MSEC(SAMPLE_INTERVAL_MS));
}

static int volume_slider_init(void) {
    if (!adc_is_ready_dt(&adc_chan)) {
        LOG_ERR("ADC controller not ready");
        return -ENODEV;
    }
    int err = adc_channel_setup_dt(&adc_chan);
    if (err) {
        LOG_ERR("adc_channel_setup_dt failed: %d", err);
        return err;
    }
    err = adc_sequence_init_dt(&adc_chan, &adc_seq);
    if (err) {
        LOG_ERR("adc_sequence_init_dt failed: %d", err);
        return err;
    }
    k_work_schedule(&volume_work, K_MSEC(500));
    LOG_INF("Volume slider initialized on AIN5 (P0.29)");
    return 0;
}

/* =========================================================
 *  初始化
 * ========================================================= */
static int ble_toggle_init(void) {
    k_work_init(&cw_work,  cw_work_handler);
    k_work_init(&ccw_work, ccw_work_handler);

    volume_slider_init();
    return 0;
}
SYS_INIT(ble_toggle_init, APPLICATION, 99);
