# Custom Macropad - ZMK Firmware

Custom 6-key macropad with rotary encoder, RGB underglow, and indicator LEDs.
Built on nRF52840 (nice!nano) with ZMK Firmware.

## Hardware

| Component | Pin | Description |
|-----------|-----|-------------|
| Key 1-6 | D3-D8 | Direct GPIO keys |
| WS2812 RGB x6 | D2 | SPI LED strip |
| White LED | D20 | Status indicator |
| Blue LED | D19 | BT indicator |
| Encoder A/B | D18/D15 | Rotary encoder |

## Flashing

1. Download `zmk.uf2` from Actions artifacts
2. Double-press reset on nice!nano
3. Drag & drop onto the USB drive
