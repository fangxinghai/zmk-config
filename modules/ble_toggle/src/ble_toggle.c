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

/* 编码器状态 */
static int  enc_confirmed_dir = 0;     /* 已确认的方向: +1=CW, -1=CCW, 0=初始 */
static int  enc_pending_dir = 0;       /* 待确认的新方向 (反转时暂存) */
static int  enc_pending_count = 0;     /* 待确认方向已连续收到的 event 次数 */
static int  enc_fire_counter = 0;      /* 已确认方向的触发计数 */

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
                /* 等凑够 EVENTS_PER_CLICK 再触发 */
                if (enc_fire_counter >= ENCODER_EVENTS_PER_CLICK) {
                    enc_fire_counter = 0;
                    encoder_fire(direction);
                }
                return ZMK_EV_EVENT_HANDLED;
            }

            /* ---- 同方向：正常计数 ---- */
            if (direction == enc_confirmed_dir) {
                /* 如果之前有待确认的反转，说明那是假反转，丢弃 */
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
                /* 已经在确认这个反转方向，继续累积 */
                enc_pending_count++;
            } else {
                /* 新的反转方向，开始计数 */
                enc_pending_dir = direction;
                enc_pending_count = 1;
            }

            LOG_DBG("Encoder: reverse pending dir=%d, count=%d/%d",
                    direction, enc_pending_count, REVERSE_CONFIRM_COUNT);

            /* 达到确认阈值：真正反转 */
            if (enc_pending_count >= REVERSE_CONFIRM_COUNT) {
                LOG_INF("Encoder: reverse confirmed %d -> %d",
                        enc_confirmed_dir, enc_pending_dir);

                enc_confirmed_dir = enc_pending_dir;
                enc_fire_counter = enc_pending_count;  /* 把确认期间的计数带过来 */
                enc_pending_dir = 0;
                enc_pending_count = 0;

                /* 检查是否已经凑够触发阈值 */
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
