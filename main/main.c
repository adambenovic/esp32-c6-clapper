#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "zboss_api.h"
#include "zboss_api_addons.h"
#include "zb_zcl.h"
#include "zb_zdo.h"

#define SOUND_SENSOR_PIN GPIO_NUM_4
#define BUTTON_PIN       GPIO_NUM_22
#define LED_PIN          GPIO_NUM_5
#define DEBOUNCE_US      (500 * 1000)

static const char *TAG = "ZB_CLAPPER";
static zb_uint8_t endpoint = 1;
static QueueHandle_t sound_evt_queue = NULL;

static void zb_zcl_on_off_set_value(zb_uint8_t param) {
    static bool led_state = false;
    led_state = !led_state;
    gpio_set_level(LED_PIN, led_state);
    ESP_LOGI(TAG, "Zigbee On/Off command received, LED state: %s", led_state ? "ON" : "OFF");
    if (param) {
        zb_buf_free(param);
    }
}

static void zb_device_cb(zb_bufid_t bufid) {
    zb_zdo_app_signal_hdr_t *sig_hnd = ZB_BUF_GET_PARAM(bufid, zb_zdo_app_signal_hdr_t);
    zb_zdo_app_signal_type_t sig = sig_hnd->signal;
    if (sig == ZB_ZDO_SIGNAL_DEVICE_ANNCE) {
        ESP_LOGI(TAG, "Device announced on Zigbee network");
    }
}

static void zb_toggle_on_off_cb(zb_uint8_t param) {
    zb_bufid_t buf = zb_buf_get_out();
    if (buf) {
        ZB_ZCL_GENERAL_SEND_ON_OFF(buf, endpoint, ZB_ZCL_CMD_ON_OFF_TOGGLE_ID);
        zb_zcl_finish_and_send(buf);
    } else {
        ESP_LOGW(TAG, "No Zigbee buffer available for toggle");
    }
}

static void IRAM_ATTR sound_isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int64_t now = esp_timer_get_time();
    xQueueSendFromISR(sound_evt_queue, &now, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void sound_sensor_task(void *arg) {
    int64_t evt_time = 0;
    int64_t last_triggered = 0;
    while (1) {
        if (xQueueReceive(sound_evt_queue, &evt_time, portMAX_DELAY)) {
            if ((evt_time - last_triggered) >= DEBOUNCE_US) {
                last_triggered = evt_time;
                ESP_LOGI(TAG, "Clap detected! Toggling LED");
                ZB_SCHEDULE_APP_CALLBACK(zb_toggle_on_off_cb, 0);
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
                ESP_LOGI(TAG, "Button pressed! Toggling LED");
                ZB_SCHEDULE_APP_CALLBACK(zb_toggle_on_off_cb, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void zb_init() {
    ESP_LOGI(TAG, "Initializing Zigbee Stack");
    ZB_INIT("ZB_CLAPPER");
    ZB_AF_REGISTER_DEVICE_CTX();
    ZB_ZCL_REGISTER_DEVICE_CB(zb_device_cb);
    zb_set_rx_on_when_idle(ZB_TRUE);
    ZB_AF_SET_ENDPOINT_HANDLER(endpoint, zb_zcl_on_off_set_value);
    zb_start();
}

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    sound_evt_queue = xQueueCreate(4, sizeof(int64_t));
    configASSERT(sound_evt_queue);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Input GPIO config: sound sensor + button
    gpio_config_t input_conf = {
        .pin_bit_mask    = (1ULL << SOUND_SENSOR_PIN) | (1ULL << BUTTON_PIN),
        .mode            = GPIO_MODE_INPUT,
        .pull_up_en      = GPIO_PULLUP_ENABLE,
        .pull_down_en    = GPIO_PULLDOWN_DISABLE,
        .intr_type       = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&input_conf));

    ESP_ERROR_CHECK(gpio_set_intr_type(SOUND_SENSOR_PIN, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SOUND_SENSOR_PIN, sound_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(SOUND_SENSOR_PIN));

    // Output GPIO config: LED
    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&output_conf));
    gpio_set_level(LED_PIN, 0);

    zb_init();
    xTaskCreate(sound_sensor_task, "sound_sensor_task", 4096, NULL, 10, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
}
