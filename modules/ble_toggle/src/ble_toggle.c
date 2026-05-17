#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zmk/event_manager.h>
#include <zmk/endpoints.h>
#include <zmk/ble.h>
#include <zmk/usb.h>

static bool ble_disabled = false;

static void disable_ble(void) {
    if (!ble_disabled) {
        bt_le_adv_stop();
        ble_disabled = true;
    }
}

static void enable_ble(void) {
    if (ble_disabled) {
        zmk_ble_clear_bonds();
        zmk_ble_prof_select(0);
        ble_disabled = false;
    }
}

static void check_output_mode(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(output_check_work, check_output_mode);

static void check_output_mode(struct k_work *work) {
    enum zmk_endpoint endpoint = zmk_endpoints_selected().transport;

    if (endpoint == ZMK_TRANSPORT_USB) {
        disable_ble();
    } else {
        enable_ble();
    }

    k_work_schedule(&output_check_work, K_SECONDS(2));
}

static int ble_toggle_init(void) {
    k_work_schedule(&output_check_work, K_SECONDS(5));
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
