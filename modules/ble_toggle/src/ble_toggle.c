#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zmk/ble.h>
#include <zmk/usb.h>
#include <zmk/endpoints_types.h>
#include <zmk/endpoints.h>

static bool ble_forced_off = false;
static bool user_selected_ble = false;

static void disable_ble(void) {
    if (!ble_forced_off) {
        bt_le_adv_stop();
        ble_forced_off = true;
    }
}

static void enable_ble(void) {
    if (ble_forced_off) {
        ble_forced_off = false;
        /* 重新启动蓝牙广播 */
        int err = bt_le_adv_start(
            BT_LE_ADV_CONN_ONE_TIME,
            NULL, 0, NULL, 0);
        if (err) {
            /* 如果上面方法失败，尝试选择配置文件恢复广播 */
            zmk_ble_prof_select(0);
        }
    }
}

static void check_output_mode(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(output_check_work, check_output_mode);

static void check_output_mode(struct k_work *work) {
    struct zmk_endpoint_instance endpoint = zmk_endpoint_get_selected();

    if (endpoint.transport == ZMK_TRANSPORT_USB) {
        /* 用户选择了 USB 模式 */
        if (!user_selected_ble) {
            disable_ble();
        }
    } else {
        /* 用户选择了 BLE 模式 */
        user_selected_ble = true;
        enable_ble();
    }

    /* 检测用户是否切回 USB */
    if (endpoint.transport == ZMK_TRANSPORT_USB) {
        user_selected_ble = false;
    }

    k_work_schedule(&output_check_work, K_SECONDS(2));
}

static int ble_toggle_init(void) {
    /* 开机延迟 5 秒后开始检测 */
    k_work_schedule(&output_check_work, K_SECONDS(5));
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
