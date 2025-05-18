/*
 * ESP32 Scroll Wheel HID – ESP‑IDF v5.x (Bluetooth Classic HID Device)
 * Hardware:
 *   GPIO16 -> Encoder A (CLK)
 *   GPIO17 -> Encoder B (DT)
 *   GPIO15 -> Encoder button (SW)
 *
 * Behaviour:
 *   • Rotate without pressing  → vertical scroll
 *   • Rotate while pressed     → horizontal scroll
 *
 * HID descriptor includes Resolution Multiplier (×4) for pixel‑level precision.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_hidd_api.h>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "SCROLL";

/* ========================== HID Descriptor ========================== */
static const uint8_t hid_report_map[] = {
    0x05, 0x01, /* USAGE_PAGE (Generic Desktop)           */
    0x09, 0x02, /* USAGE (Mouse)                          */
    0xA1, 0x01, /* COLLECTION (Application)               */
    0x85, 0x01, /*   REPORT_ID (1)                        */

    0x09, 0x01, /*   USAGE (Pointer)                      */
    0xA1, 0x00, /*   COLLECTION (Physical)                */

    /* 3 buttons */
    0x05, 0x09,
    0x19, 0x01,
    0x29, 0x03,
    0x15, 0x00,
    0x25, 0x01,
    0x95, 0x03,
    0x75, 0x01,
    0x81, 0x02,
    0x95, 0x01,
    0x75, 0x05,
    0x81, 0x03,

    /* X/Y (unused) */
    0x05, 0x01,
    0x09, 0x30,
    0x09, 0x31,
    0x15, 0x81,
    0x25, 0x7F,
    0x75, 0x08,
    0x95, 0x02,
    0x81, 0x06,

    /* Vertical wheel + Resolution Multiplier */
    0xA1, 0x02,
    0x09, 0x48, /* Resolution Multiplier */
    0x15, 0x00,
    0x25, 0x01,
    0x35, 0x01,
    0x45, 0x04, /* ×4 */
    0x75, 0x02,
    0x95, 0x01,
    0xB1, 0x02, /* FEATURE */

    0x09, 0x38, /* Wheel */
    0x15, 0x81,
    0x25, 0x7F,
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x06, /* INPUT (Rel) */
    0xC0,

    /* Horizontal wheel + Resolution Multiplier */
    0xA1, 0x02,
    0x09, 0x48,
    0x15, 0x00,
    0x25, 0x01,
    0x35, 0x01,
    0x45, 0x04,
    0x75, 0x02,
    0x95, 0x01,
    0xB1, 0x02,
    0x75, 0x04,
    0x95, 0x01,
    0xB1, 0x03,
    0x05, 0x0C,       /* Consumer */
    0x0A, 0x38, 0x02, /* AC Pan */
    0x15, 0x81,
    0x25, 0x7F,
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x06,
    0xC0,

    0xC0,
    0xC0};

/* =================== Static HID app & QoS params ==================== */
static esp_hidd_app_param_t app_param = {
    .name = "ESP32 Scroll Wheel",
    .description = "Scroll HID",
    .provider = "ESP32",
    .subclass = ESP_HID_CLASS_MIC, /* Mouse */
    .desc_list = (uint8_t *)hid_report_map,
    .desc_list_len = sizeof(hid_report_map)};
static esp_hidd_qos_param_t qos_param = {0};

/* ====================== Hardware pin mapping ======================== */
#define PIN_ENC_A 16
#define PIN_ENC_B 17
#define PIN_BTN 15

/* ================= Encoder state & lookup table ==================== */
static const int8_t enc_lut[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};
static volatile int8_t quad_accum = 0; /* counts every edge */
static volatile int8_t wheel_vert = 0; /* +/‑1 per detent */
static volatile int8_t wheel_horz = 0;

/* ============================== ISR ================================ */
static void IRAM_ATTR encoder_isr(void *arg)
{
    static uint8_t prev = 0;
    uint8_t curr = (gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B);
    uint8_t idx = (prev << 2) | curr;
    prev = curr;

    int8_t delta = enc_lut[idx]; /* ‑1, 0, +1 */
    if (!delta)
        return; /* invalid transition */

    quad_accum += delta; /* accumulate edges */

    if (quad_accum >= +4)
    {
        quad_accum = 0;
        if (gpio_get_level(PIN_BTN) == 0)
            wheel_horz += +1;
        else
            wheel_vert += +1;
    }
    else if (quad_accum <= -4)
    {
        quad_accum = 0;
        if (gpio_get_level(PIN_BTN) == 0)
            wheel_horz -= 1;
        else
            wheel_vert -= 1;
    }
}

/* ======================== HID send task ============================ */
static void hid_send_task(void *arg)
{
    uint8_t report[5] = {0};
    while (1)
    {
        if (wheel_vert || wheel_horz)
        {
            report[3] = wheel_vert;
            report[4] = wheel_horz;
            esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA,
                                          0x01, sizeof(report), report);
            wheel_vert = wheel_horz = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); /* 100 Hz */
    }
}

/* ============================ GAP cb =============================== */
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    /* No GAP handling needed, just log */
    if (event == ESP_BT_GAP_AUTH_CMPL_EVT)
        ESP_LOGI(TAG, "Pairing OK");
}

/* ========================= HID callback ============================ */
static void hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    static bool host_connected = false;

    switch (event)
    {
    case ESP_HIDD_INIT_EVT:
        ESP_LOGI(TAG, "HID init done");
        esp_bt_hid_device_register_app(&app_param, &qos_param, &qos_param);
        break;

    case ESP_HIDD_REGISTER_APP_EVT:
        ESP_LOGI(TAG, "HID app registered – connectable");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;

    case ESP_HIDD_OPEN_EVT:
        ESP_LOGI(TAG, "Host opened (status=%d)", param->open.conn_status);
        host_connected = (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED);
        break;

    case ESP_HIDD_CLOSE_EVT:
        ESP_LOGI(TAG, "Host closed (status=%d)", param->close.conn_status);
        host_connected = false;
        break;

    default:
        break;
    }
}

/* =============================== main ============================== */
void app_main(void)
{
    /* NVS */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Bluetooth controller */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)); /* Classic only */

    /* Bluedroid stack */
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    /* Register callbacks */
    ESP_ERROR_CHECK(esp_bt_hid_device_register_callback(hidd_cb));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_bt_hid_device_init());

    /* Device name */
    esp_bt_gap_set_device_name("ESP32 Scroll Wheel");

    /* GPIO config */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B) | (1ULL << PIN_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io);

    /* ISR */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_A, encoder_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_B, encoder_isr, NULL);

    /* HID send task */
    xTaskCreatePinnedToCore(hid_send_task, "hid_send", 2048, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "Setup complete - ready to pair");
}
