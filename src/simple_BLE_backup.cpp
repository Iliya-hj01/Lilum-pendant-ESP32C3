#include "simple_BLE.h"

#include <string.h>
#include <assert.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_nimble_hci.h"
#include "esp_bt.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#include "sketch.h"   // should declare: extern volatile uint8_t g_animation_mode;

static const char *TAG = "BLE";
static bool ble_initialized = false;
static bool ble_advertising = false;
static bool ble_connected = false;

// Custom 128-bit UUIDs
static const ble_uuid128_t svc_uuid = {
    { BLE_UUID_TYPE_128 },
    { 0xab, 0x90, 0x78, 0x56,
      0x34, 0x12, 0x34, 0x12,
      0x34, 0x12, 0x34, 0x12,
      0x78, 0x56, 0x34, 0x12 }
};

static const ble_uuid128_t chr_uuid = {
    { BLE_UUID_TYPE_128 },
    { 0xac, 0x90, 0x78, 0x56,
      0x34, 0x12, 0x34, 0x12,
      0x34, 0x12, 0x34, 0x12,
      0x78, 0x56, 0x34, 0x12 }
};

static uint16_t mode_chr_handle;
static uint8_t own_addr_type;
static TimerHandle_t adv_restart_timer = NULL;  // Timer used to restart advertising at intervals

// Forward declarations
static void ble_app_advertise(void);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int mode_chr_access(uint16_t conn_handle,
                           uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg);
static void ble_app_on_sync(void);
static void ble_app_on_reset(int reason);
static void ble_host_task(void *param);
static void adv_restart_timer_cb(TimerHandle_t xTimer);

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &chr_uuid.u,
                .access_cb = mode_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .val_handle = &mode_chr_handle,
            },
            {0} // end of characteristics
        },
    },
    {0} // end of services
};

static int mode_chr_access(uint16_t conn_handle,
                           uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            int rc = os_mbuf_append(ctxt->om,
                                    reinterpret_cast<const void *>(&g_animation_mode),
                                    sizeof(g_animation_mode));
            if (rc != 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }

            ESP_LOGI(TAG, "Read mode = %u", g_animation_mode);
            return 0;
        }

        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            uint8_t new_mode = 0;
            uint16_t out_len = 0;

            if (OS_MBUF_PKTLEN(ctxt->om) != 1) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            int rc = ble_hs_mbuf_to_flat(ctxt->om,
                                         &new_mode,
                                         sizeof(new_mode),
                                         &out_len);
            if (rc != 0 || out_len != 1) {
                return BLE_ATT_ERR_UNLIKELY;
            }

            g_animation_mode = new_mode;
            ESP_LOGI(TAG, "Wrote mode = %u", g_animation_mode);
            return 0;
        }

        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ble_connected = true;
                ble_advertising = false;
                ESP_LOGI(TAG, "BLE connected");
            } else {
                ESP_LOGW(TAG, "BLE connect failed; restarting advertising");
                ble_app_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ble_connected = false;
            ESP_LOGI(TAG, "BLE disconnected; reason=%d", event->disconnect.reason);
            ble_app_advertise();
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_advertising = false;
            ESP_LOGI(TAG, "Advertising complete; will restart in 38 seconds");
    // Allow BLE controller to sleep while idle
    esp_bt_sleep_enable();
    if (adv_restart_timer) {
        xTimerStart(adv_restart_timer, 0);
    }
    return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event");
            return 0;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU updated; value=%d", event->mtu.value);
            return 0;

        default:
            return 0;
    }
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;

    memset(&fields, 0, sizeof(fields));

    // Advertise device name
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;

    // General discoverable + BLE only
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // Ensure the controller is awake before starting advertising
    esp_bt_sleep_disable();

    rc = ble_gap_adv_start(own_addr_type,
                           NULL,
                           2000,  // Advertise for 2 seconds
                           &adv_params,
                           ble_gap_event,
                           NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ble_advertising = true;
    ESP_LOGI(TAG, "Advertising started");
}

static void ble_app_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG,
                 "BLE address: %02X:%02X:%02X:%02X:%02X:%02X",
                 addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0]);
    }

    ble_app_advertise();
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset; reason=%d", reason);
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void adv_restart_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    ESP_LOGI(TAG, "Advertising restart timer fired");
    ble_app_advertise();
}

void ble_init(void)
{
    int rc;
    ESP_LOGI(TAG, "Initializing BLE");
    ESP_LOGI(TAG, "Initializing BLE controller");
    esp_nimble_hci_and_controller_init();
    ESP_LOGI(TAG, "BLE controller initialized");

    // Enable BLE modem sleep when idle
    esp_bt_sleep_enable();

    nimble_port_init();
    ESP_LOGI(TAG, "NimBLE port initialized");
    // Standard GAP/GATT services
    ble_svc_gap_init();
    ESP_LOGI(TAG, "GAP service initialized");
    ble_svc_gatt_init();
    ESP_LOGI(TAG, "GATT service initialized");
    // Device name shown in scanner apps
    rc = ble_svc_gap_device_name_set("Pendant-01");
    assert(rc == 0);

    // Host callbacks
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // Register our GATT database
    rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    // Storage helper for bonds / keys / CCCD state
    // (Not required on this IDF version; configured internally)

    // Create timer that restarts advertising after a pause
    adv_restart_timer = xTimerCreate("adv_restart",
                                    pdMS_TO_TICKS(38000), // 38 seconds pause
                                    pdFALSE,
                                    NULL,
                                    adv_restart_timer_cb);
    if (!adv_restart_timer) {
        ESP_LOGE(TAG, "Failed to create advertising restart timer");
    }

    // Start NimBLE host task
    nimble_port_freertos_init(ble_host_task);

    ble_initialized = true;
    ESP_LOGI(TAG, "BLE init complete");
}

