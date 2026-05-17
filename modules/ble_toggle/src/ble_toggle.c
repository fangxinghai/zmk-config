#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zmk/event_manager.h>
#include <zmk/events/output_selection_changed.h>
#include <zmk/ble.h>
#include <zmk/usb.h>

static void disable_ble(void) {
    bt_conn_foreach(BT_CONN_TYPE_LE,
                   (bt_conn_foreach_cb)bt_conn_disconnect,
                   BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    bt_le_adv_stop();
}

static void enable_ble(void) {
    zmk_ble_adv_resume();
}

static int output_selection_listener(const zmk_event_t *eh) {
    const struct zmk_output_selection_changed *ev =
        as_zmk_output_selection_changed(eh);

    if (ev == NULL) return 0;

    if (ev->transport == ZMK_OUTPUT_TRANSPORT_USB) {
        disable_ble();
    } else if (ev->transport == ZMK_OUTPUT_TRANSPORT_BLE) {
        enable_ble();
    }
    return 0;
}

ZMK_LISTENER(ble_toggle, output_selection_listener);
ZMK_SUBSCRIPTION(ble_toggle, zmk_output_selection_changed);

static int ble_toggle_init(void) {
    if (zmk_usb_is_powered()) {
        disable_ble();
    } else {
        enable_ble();
    }
    return 0;
}

SYS_INIT(ble_toggle_init, APPLICATION, 99);
