# Clapper Firmware Refactor Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Fix all bugs and refactor main.c for stability and efficiency on ESP32-C6 Zigbee router clapper.

**Architecture:** Single-file firmware refactor. Sound sensor moves from polling to ISR+queue with timestamp debounce. Button keeps polling but uses non-blocking timestamp debounce. All bugs fixed. No new features.

**Tech Stack:** ESP-IDF 5.4.0, FreeRTOS, ZBOSS Zigbee 1.6.4, ESP32-C6 (RISC-V)

---

### Task 1: Fix double nvs_flash_init

**Files:**
- Modify: `main/main.c`

**Step 1: Remove second call**

In `app_main`, there are two `nvs_flash_init()` calls. The second one (line 91, after `gpio_config`) has no error handling and is redundant. Delete it.

Before:
```c
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(nvs_flash_init());  // <-- DELETE THIS LINE

    zb_init();
```

After:
```c
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    zb_init();
```

**Step 2: Verify build**

```bash
idf.py build
```
Expected: compiles with no errors.

---

### Task 2: Split GPIO config — INPUT vs OUTPUT

**Files:**
- Modify: `main/main.c`

**Step 1: Replace the bulk gpio_config with two separate configs**

Current code configures all 3 pins as INPUT then overrides LED to OUTPUT — wrong pattern.

Replace:
```c
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SOUND_SENSOR_PIN) | (1ULL << BUTTON_PIN) | (1ULL << LED_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
```

With:
```c
    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << SOUND_SENSOR_PIN) | (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_conf));

    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_conf));
    gpio_set_level(LED_PIN, 0);
```

**Step 2: Verify build**

```bash
idf.py build
```
Expected: compiles with no errors.

---

### Task 3: Rename and fix zb_toggle_on_off

**Files:**
- Modify: `main/main.c`

**Step 1: Replace zb_send_on_off with clean zb_toggle_on_off**

The old function had an unused `cmd_info` variable and ignored the `on` parameter. Since the clapper always toggles, remove the bool param entirely.

Replace:
```c
static void zb_send_on_off(bool on) {
    zb_zcl_parsed_hdr_t *cmd_info;
    zb_zcl_on_off_send_t on_off_cmd;
    on_off_cmd.on_off = on ? ZB_ZCL_CMD_ON_OFF_ON_ID : ZB_ZCL_CMD_ON_OFF_OFF_ID;
    zb_bufid_t buf = zb_buf_get_out();
    if (buf) {
        ZB_ZCL_GENERAL_SEND_ON_OFF(buf, endpoint, ZB_ZCL_CMD_ON_OFF_TOGGLE_ID);
        zb_zcl_finish_and_send(buf);
    }
}
```

With:
```c
static void zb_toggle_on_off(void) {
    zb_bufid_t buf = zb_buf_get_out();
    if (buf) {
        ZB_ZCL_GENERAL_SEND_ON_OFF(buf, endpoint, ZB_ZCL_CMD_ON_OFF_TOGGLE_ID);
        zb_zcl_finish_and_send(buf);
    } else {
        ESP_LOGW(TAG, "No Zigbee buffer available for toggle");
    }
}
```

**Step 2: Update all call sites**

Change `zb_send_on_off(1)` → `zb_toggle_on_off()` in both `sound_sensor_task` and `button_task`.

**Step 3: Verify build**

```bash
idf.py build
```
Expected: no errors, no unused variable warnings.

---

### Task 4: Sound sensor — ISR + queue + timestamp debounce

**Files:**
- Modify: `main/main.c`

**Step 1: Add queue handle and ISR at top of file (after TAG definition)**

```c
static QueueHandle_t sound_evt_queue = NULL;

static void IRAM_ATTR sound_isr_handler(void *arg) {
    int64_t now = esp_timer_get_time();
    xQueueSendFromISR(sound_evt_queue, &now, NULL);
}
```

Note: `esp_timer_get_time()` is IRAM-safe and returns microseconds since boot.

**Step 2: Create the queue in app_main before GPIO config**

```c
    sound_evt_queue = xQueueCreate(4, sizeof(int64_t));
    configASSERT(sound_evt_queue);
```

**Step 3: Enable interrupt on sound sensor pin after INPUT gpio_config**

Add after `ESP_ERROR_CHECK(gpio_config(&input_conf))`:

```c
    gpio_isr_handler_add(SOUND_SENSOR_PIN, sound_isr_handler, NULL);
    gpio_set_intr_type(SOUND_SENSOR_PIN, GPIO_INTR_POSEDGE);
    gpio_intr_enable(SOUND_SENSOR_PIN);
```

And install the GPIO ISR service before the queue create (once per app):

```c
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
```

**Step 4: Replace sound_sensor_task with queue-based + timestamp debounce**

```c
#define DEBOUNCE_US (500 * 1000)  // 500ms in microseconds

void sound_sensor_task(void *arg) {
    int64_t last_triggered = 0;
    int64_t evt_time;
    while (1) {
        if (xQueueReceive(sound_evt_queue, &evt_time, portMAX_DELAY)) {
            if ((evt_time - last_triggered) >= DEBOUNCE_US) {
                last_triggered = evt_time;
                ESP_LOGI(TAG, "Clap detected! Toggling Zigbee");
                zb_toggle_on_off();
            }
        }
    }
}
```

**Step 5: Verify build**

```bash
idf.py build
```
Expected: no errors. Check for `esp_timer.h` — add `#include "esp_timer.h"` at top if needed.

---

### Task 5: Button task — non-blocking timestamp debounce

**Files:**
- Modify: `main/main.c`

**Step 1: Replace blocking debounce with timestamp check**

```c
void button_task(void *arg) {
    int64_t last_triggered = 0;
    while (1) {
        if (gpio_get_level(BUTTON_PIN) == 0) {
            int64_t now = esp_timer_get_time();
            if ((now - last_triggered) >= DEBOUNCE_US) {
                last_triggered = now;
                ESP_LOGI(TAG, "Button pressed! Toggling Zigbee");
                zb_toggle_on_off();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

Note: Poll interval reduced 100ms → 10ms since debounce is now timestamp-based. Button press will feel more responsive.

**Step 2: Verify build**

```bash
idf.py build
```
Expected: no errors.

---

### Task 6: Bump task stack sizes 2048 → 4096

**Files:**
- Modify: `main/main.c`

**Step 1: Update xTaskCreate calls in app_main**

```c
    xTaskCreate(sound_sensor_task, "sound_sensor_task", 4096, NULL, 10, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
```

**Step 2: Final build**

```bash
idf.py build
```
Expected: clean build, no warnings.

**Step 3: Flash and verify on hardware**

```bash
idf.py flash monitor
```

Expected serial output on boot:
```
I (xxx) ZB_CLAPPER: Initializing Zigbee Stack
```

On clap (GPIO4 rising edge):
```
I (xxx) ZB_CLAPPER: Clap detected! Toggling Zigbee
```

On button press (GPIO22 low):
```
I (xxx) ZB_CLAPPER: Button pressed! Toggling Zigbee
```

---

### Final file state: main/main.c

Complete file for reference after all tasks:

```c
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "zboss_api.h"
#include "zboss_api_addons.h"
#include "zb_zcl.h"
#include "zb_zdo.h"

#define SOUND_SENSOR_PIN  GPIO_NUM_4
#define BUTTON_PIN        GPIO_NUM_22
#define LED_PIN           GPIO_NUM_5
#define DEBOUNCE_US       (500 * 1000)

static const char *TAG = "ZB_CLAPPER";
static zb_uint8_t endpoint = 1;
static QueueHandle_t sound_evt_queue = NULL;

static void IRAM_ATTR sound_isr_handler(void *arg) {
    int64_t now = esp_timer_get_time();
    xQueueSendFromISR(sound_evt_queue, &now, NULL);
}

static void zb_zcl_on_off_set_value(zb_uint8_t param) {
    static bool led_state = false;
    led_state = !led_state;
    gpio_set_level(LED_PIN, led_state);
    ESP_LOGI(TAG, "Zigbee On/Off command received, LED: %s", led_state ? "ON" : "OFF");
}

static void zb_device_cb(zb_bufid_t bufid) {
    zb_zdo_app_signal_hdr_t *sig_hnd = ZB_BUF_GET_PARAM(bufid, zb_zdo_app_signal_hdr_t);
    zb_zdo_app_signal_type_t sig = sig_hnd->signal;
    if (sig == ZB_ZDO_SIGNAL_DEVICE_ANNCE) {
        ESP_LOGI(TAG, "Device announced on Zigbee network");
    }
}

static void zb_toggle_on_off(void) {
    zb_bufid_t buf = zb_buf_get_out();
    if (buf) {
        ZB_ZCL_GENERAL_SEND_ON_OFF(buf, endpoint, ZB_ZCL_CMD_ON_OFF_TOGGLE_ID);
        zb_zcl_finish_and_send(buf);
    } else {
        ESP_LOGW(TAG, "No Zigbee buffer available for toggle");
    }
}

void sound_sensor_task(void *arg) {
    int64_t last_triggered = 0;
    int64_t evt_time;
    while (1) {
        if (xQueueReceive(sound_evt_queue, &evt_time, portMAX_DELAY)) {
            if ((evt_time - last_triggered) >= DEBOUNCE_US) {
                last_triggered = evt_time;
                ESP_LOGI(TAG, "Clap detected! Toggling Zigbee");
                zb_toggle_on_off();
            }
        }
    }
}

void button_task(void *arg) {
    int64_t last_triggered = 0;
    while (1) {
        if (gpio_get_level(BUTTON_PIN) == 0) {
            int64_t now = esp_timer_get_time();
            if ((now - last_triggered) >= DEBOUNCE_US) {
                last_triggered = now;
                ESP_LOGI(TAG, "Button pressed! Toggling Zigbee");
                zb_toggle_on_off();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void zb_init(void) {
    ESP_LOGI(TAG, "Initializing Zigbee Stack");
    ZB_INIT("ZB_CLAPPER");
    ZB_AF_REGISTER_DEVICE_CTX();
    ZB_ZCL_REGISTER_DEVICE_CB(zb_device_cb);
    zb_set_rx_on_when_idle(ZB_TRUE);
    ZB_AF_SET_ENDPOINT_HANDLER(endpoint, zb_zcl_on_off_set_value);
    zb_start();
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    sound_evt_queue = xQueueCreate(4, sizeof(int64_t));
    configASSERT(sound_evt_queue);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << SOUND_SENSOR_PIN) | (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_conf));

    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_conf));
    gpio_set_level(LED_PIN, 0);

    gpio_isr_handler_add(SOUND_SENSOR_PIN, sound_isr_handler, NULL);
    gpio_set_intr_type(SOUND_SENSOR_PIN, GPIO_INTR_POSEDGE);
    gpio_intr_enable(SOUND_SENSOR_PIN);

    zb_init();
    xTaskCreate(sound_sensor_task, "sound_sensor_task", 4096, NULL, 10, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
}
```
