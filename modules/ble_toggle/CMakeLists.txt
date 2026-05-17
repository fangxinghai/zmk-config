add_library(ble_toggle STATIC)
target_sources(ble_toggle PRIVATE src/ble_toggle.c)
target_link_libraries(ble_toggle PRIVATE zephyr_interface)
zephyr_append_cmake_library(ble_toggle)
