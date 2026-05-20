#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "ha/esp_zigbee_ha_standard.h"

#define SOUND_SENSOR_PIN    GPIO_NUM_4
#define BUTTON_PIN          GPIO_NUM_9   /* TEST: BOOT button on devkit — swap back to 22 if GPIO9 works */
#define LED_PIN             GPIO_NUM_5

/* Clap pattern: 3 claps, 150–1500 ms apart, within 4 s */
#define CLAP_DEBOUNCE_US    (150 * 1000)
#define CLAP_MAX_GAP_US     (1500 * 1000)
#define CLAP_WINDOW_US      (4000 * 1000)
#define CLAP_REQUIRED       3

#define BUTTON_DEBOUNCE_US  (50 * 1000)

#define EP_BUTTON           1
#define EP_CLAPPER          2

/* ZCL character string: byte 0 = length, rest = ASCII */
#define ZB_MANUFACTURER     "\x0B" "adambenovic"
#define ZB_MODEL_BUTTON     "\x10" "c6clapper-button"
#define ZB_MODEL_CLAPPER    "\x0F" "c6clapper-sound"

static const char *TAG = "ZB_CLAPPER";
static QueueHandle_t sound_evt_queue = NULL;
static bool button_state  = false;
static bool clapper_state = false;
static bool zb_started    = false;

/* ── Zigbee stack callbacks ──────────────────────────────────────────── */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            zb_started = true;
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Zigbee init failed: %s", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network, addr: 0x%04hx", esp_zb_get_short_address());
        } else {
            ESP_LOGW(TAG, "Steering failed, retrying...");
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        break;
    }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        const esp_zb_zcl_set_attr_value_message_t *msg = message;
        if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
            msg->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
            bool on = *(uint8_t *)msg->attribute.data.value;
            if (msg->info.dst_endpoint == EP_BUTTON) {
                button_state = on;
            } else if (msg->info.dst_endpoint == EP_CLAPPER) {
                clapper_state = on;
            }
            gpio_set_level(LED_PIN, button_state || clapper_state);
        }
    }
    return ESP_OK;
}

/* ── Zigbee attribute update + report ───────────────────────────────── */

static void zb_set_on_off(uint8_t ep, bool on)
{
    if (!zb_started) return;
    uint8_t value = on ? 1 : 0;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(ep,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
        &value, false);
    esp_zb_lock_release();
}

/* ── ISR + tasks ─────────────────────────────────────────────────────── */

static void IRAM_ATTR sound_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int64_t now = esp_timer_get_time();
    xQueueSendFromISR(sound_evt_queue, &now, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void sound_sensor_task(void *arg)
{
    int64_t evt_time = 0;
    int64_t clap_times[CLAP_REQUIRED] = {0};
    int     clap_count = 0;
    int64_t last_evt   = 0;

    while (1) {
        if (!xQueueReceive(sound_evt_queue, &evt_time, portMAX_DELAY)) continue;

        /* Global debounce — drop bursts regardless of clap_count */
        if ((evt_time - last_evt) < CLAP_DEBOUNCE_US) continue;
        last_evt = evt_time;

        /* Gap too large — pattern broken, start over */
        if (clap_count > 0 && (evt_time - clap_times[clap_count - 1]) > CLAP_MAX_GAP_US) {
            ESP_LOGD(TAG, "Gap too large, reset");
            clap_count = 0;
        }

        /* Window expired — reset */
        if (clap_count > 0 && (evt_time - clap_times[0]) > CLAP_WINDOW_US) {
            ESP_LOGD(TAG, "Window expired, reset");
            clap_count = 0;
        }

        clap_times[clap_count++] = evt_time;
        ESP_LOGI(TAG, "Clap %d/%d", clap_count, CLAP_REQUIRED);

        if (clap_count >= CLAP_REQUIRED) {
            clap_count = 0;
            clapper_state = !clapper_state;
            gpio_set_level(LED_PIN, button_state || clapper_state);
            ESP_LOGI(TAG, "3-clap! Clapper %s", clapper_state ? "ON" : "OFF");
            zb_set_on_off(EP_CLAPPER, clapper_state);

            /* 3 s lockout: drain noise, sleep, drain again, reset debounce */
            int64_t drain;
            while (xQueueReceive(sound_evt_queue, &drain, 0) == pdTRUE) {}
            vTaskDelay(pdMS_TO_TICKS(3000));
            while (xQueueReceive(sound_evt_queue, &drain, 0) == pdTRUE) {}
            last_evt = esp_timer_get_time();
        }
    }
}

static void button_task(void *arg)
{
    int64_t last_triggered = 0;
    int last_level = -1;
    int alive_ctr = 0;
    while (1) {
        int level = gpio_get_level(BUTTON_PIN);

        if (++alive_ctr >= 200) {          /* log every 2 s to prove task alive */
            alive_ctr = 0;
            ESP_LOGI(TAG, "button_task alive, GPIO%d=%d", BUTTON_PIN, level);
        }
        if (level != last_level) {
            ESP_LOGI(TAG, "Button GPIO%d level -> %d", BUTTON_PIN, level);
            last_level = level;
        }

        if (level == 0) {
            int64_t now = esp_timer_get_time();
            if ((now - last_triggered) >= BUTTON_DEBOUNCE_US) {
                last_triggered = now;
                button_state = !button_state;
                gpio_set_level(LED_PIN, button_state || clapper_state);
                ESP_LOGI(TAG, "Button! EP %s", button_state ? "ON" : "OFF");
                zb_set_on_off(EP_BUTTON, button_state);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── Zigbee init ─────────────────────────────────────────────────────── */

static esp_zb_ep_list_t *create_endpoints(void)
{
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,
    };
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {.on_off = 0};
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    /* EP_BUTTON — build from scratch: avoids duplicate basic attrs */
    esp_zb_attribute_list_t *btn_basic = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_basic_cluster_add_attr(btn_basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                  ZB_MANUFACTURER);
    esp_zb_basic_cluster_add_attr(btn_basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                  ZB_MODEL_BUTTON);
    esp_zb_cluster_list_t *btn_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(btn_clusters, btn_basic,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(btn_clusters,
        esp_zb_on_off_cluster_create(&on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_ep_list_add_ep(ep_list, btn_clusters, (esp_zb_endpoint_config_t){
        .endpoint        = EP_BUTTON,
        .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id   = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    });

    /* EP_CLAPPER */
    esp_zb_attribute_list_t *clap_basic = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_basic_cluster_add_attr(clap_basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                  ZB_MANUFACTURER);
    esp_zb_basic_cluster_add_attr(clap_basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                  ZB_MODEL_CLAPPER);
    esp_zb_cluster_list_t *clap_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(clap_clusters, clap_basic,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(clap_clusters,
        esp_zb_on_off_cluster_create(&on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_ep_list_add_ep(ep_list, clap_clusters, (esp_zb_endpoint_config_t){
        .endpoint        = EP_CLAPPER,
        .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id   = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    });

    return ep_list;
}

static void zb_task(void *pvParameters)
{
    esp_zb_platform_config_t platform_cfg = {
        .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
        .host_config  = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_cfg));

    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg.max_children = 10,
    };
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_device_register(create_endpoints());
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/* ── app_main ────────────────────────────────────────────────────────── */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << SOUND_SENSOR_PIN) | (1ULL << BUTTON_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_conf));

    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_conf));
    gpio_set_level(LED_PIN, 0);

    sound_evt_queue = xQueueCreate(8, sizeof(int64_t));
    configASSERT(sound_evt_queue);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_set_intr_type(SOUND_SENSOR_PIN, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SOUND_SENSOR_PIN, sound_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_intr_enable(SOUND_SENSOR_PIN));

    xTaskCreate(zb_task,           "zb_task",     4096, NULL, 6, NULL);
    xTaskCreate(sound_sensor_task, "sound_task",  4096, NULL, 5, NULL);
    xTaskCreate(button_task,       "button_task", 4096, NULL, 5, NULL);
}
