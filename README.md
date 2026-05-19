# ESP32-C6 Zigbee Clapper

Clap-activated Zigbee switch built on the ESP32-C6. Detects sound events via a microphone/sound sensor, debounces them, and sends a ZCL On/Off Toggle command over Zigbee. A physical button provides manual override. The onboard LED mirrors the toggle state locally.

## Hardware

| Signal | GPIO |
|---|---|
| Sound sensor (digital out) | GPIO 4 |
| Manual button | GPIO 22 |
| Status LED | GPIO 5 |

- Sound sensor pin is pulled up internally; trigger on rising edge
- Button pin is pulled up internally; active low
- LED is active high

## Features

- **Clap detection** — rising-edge ISR feeds a FreeRTOS queue; task consumes with 500 ms timestamp debounce
- **Button** — 10 ms polling with 500 ms timestamp debounce
- **Zigbee router** — joins an existing network, relays traffic, sends ZCL toggle on each clap/press
- **Web flasher** — GitHub Pages hosts a one-click browser flasher (no drivers needed)

## Flashing

### Browser (easiest)

Open **[adambenovic.github.io/esp32-c6-clapper](https://adambenovic.github.io/esp32-c6-clapper)**, connect the ESP32-C6 via USB, click Flash.

### Docker (local)

```bash
# Build
make build

# Flash (default port /dev/ttyACM0)
make flash

# or specify port
make flash PORT=/dev/ttyUSB0

# Serial monitor
make monitor
```

Requires Docker. No local ESP-IDF installation needed.

## Building from source

```bash
# Build only
docker run --rm -v $(pwd):/project -w /project espressif/idf:release-v5.4 \
  bash -c "git config --global --add safe.directory /project && idf.py build"
```

## Project structure

```
main/
  main.c              — application entry point, all firmware logic
  CMakeLists.txt      — component registration
  idf_component.yml   — managed component dependencies
sdkconfig.defaults    — target (esp32c6) and Zigbee config
docs/
  index.html          — ESP Web Tools browser flasher
  manifest.json       — firmware manifest (version + URL)
  firmware-merged.bin — merged binary (published by CI on release)
.github/workflows/
  build.yml           — CI: build on every push, release on v* tag
```

## CI / Releases

Every push builds the firmware. Pushing a `v*` tag (e.g. `v1.0.0`) triggers the full release pipeline:

1. Build firmware with `espressif/idf:release-v5.4`
2. Merge bootloader + partition table + app into a single flashable `.bin`
3. Publish binary to `docs/` and update `manifest.json`
4. Create GitHub Release with the merged binary attached

## Dependencies

- [espressif/esp-zboss-lib](https://components.espressif.com/components/espressif/esp-zboss-lib) `>=1.6.4,<2.0.0`
- [espressif/esp-zigbee-lib](https://components.espressif.com/components/espressif/esp-zigbee-lib) `>=1.6.4,<2.0.0`
- ESP-IDF `>=5.4.0`
