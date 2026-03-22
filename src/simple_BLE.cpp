#include "simple_BLE.h"

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <limits.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_nimble_hci.h"

#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#include "orchestra_config.h"
#include "orchestra_payload.h"
#include "sketch.h"   // should declare: extern volatile uint8_t g_animation_mode;

static const char *TAG = "BLE";
static constexpr uint32_t BLE_STARTUP_ADV_MS = ORCHESTRA_STARTUP_WINDOW_MS;
static constexpr uint32_t BLE_CYCLE_OFF_MS = ORCHESTRA_CYCLE_OFF_MS;
static constexpr uint32_t BLE_CYCLE_ADV_MS = ORCHESTRA_CYCLE_ADV_MS;
static bool ble_initialized = false;
static bool ble_advertising = false;
static bool ble_scanning = false;
static bool ble_connected = false;
static bool ble_startup_cycle_complete = false;
static bool follower_synced = false;
static int64_t follower_master_offset_ms = 0;  // Local time minus master's timestamp.
static TaskHandle_t follower_scan_task_handle = NULL;

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
static SemaphoreHandle_t ble_host_stopped_sem = NULL;  // Signalled by host task just before it exits

// Forward declarations

static void ble_app_advertise(void); // Starts BLE advertising with the current timing window configuration.
static int ble_gap_event(struct ble_gap_event *event, void *arg); // Handles GAP events such as connect, disconnect, and advertising completion.
// Implements GATT characteristic read/write access for animation mode.
static int mode_chr_access(uint16_t conn_handle,
                           uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg);
static void ble_app_on_sync(void); // Runs when the BLE host is synchronized and ready to begin advertising.
static void ble_app_on_reset(int reason); // Logs BLE host reset reasons for diagnostics.
static void ble_host_task(void *param); // NimBLE host task entry point that runs the BLE host loop.
static void ble_deinit(void); // Deinitializes the BLE host, port, and controller stack.
static void ble_deinit_and_restart_task(void *param); // One-shot task that deinitializes BLE, waits, then reinitializes BLE. 
static uint32_t ble_get_sync_timestamp_ms(void); // Monotonic timestamp used as the sync reference in advertising.
static bool ble_is_sync_source(const OrchestraAdvPayload &payload); // Checks whether an advertisement came from the configured master.
static void ble_apply_master_packet(const OrchestraAdvPayload &payload); // Updates follower sync and animation based on master packet.
static void ble_follower_scan_scheduler_task(void *param); // Maintains scan windows aligned to the master's BLE timing.
static void ble_start_scan_for_ms(uint32_t duration_ms); // Starts a passive BLE scan for the requested duration.

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

static uint32_t ble_get_sync_timestamp_ms(void)
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

static bool ble_is_sync_source(const OrchestraAdvPayload &payload)
{
    const bool role_ok = payload.role == OrchestraRole::Master;
    const bool group_ok = payload.group_id == ORCHESTRA_GROUP_ID;
    const bool master_id_ok = payload.master_id == ORCHESTRA_MASTER_ID;
    const bool device_ok = payload.device_id == ORCHESTRA_MASTER_ID;
    return role_ok && group_ok && master_id_ok && device_ok;
}

static void ble_apply_master_packet(const OrchestraAdvPayload &payload)
{
    const uint32_t now_ms = ble_get_sync_timestamp_ms();
    follower_master_offset_ms = static_cast<int64_t>(now_ms) - static_cast<int64_t>(payload.sync_timestamp_ms);

    if (!follower_synced) {
        follower_synced = true;
        ESP_LOGI(TAG,
                 "Follower synced to master device=%u group=%u master_ts=%lu local_ts=%lu",
                 payload.device_id,
                 payload.group_id,
                 static_cast<unsigned long>(payload.sync_timestamp_ms),
                 static_cast<unsigned long>(now_ms));
    }

    if (g_animation_mode != payload.animation_mode) {
        g_animation_mode = payload.animation_mode;
        ESP_LOGI(TAG, "Animation updated by master to mode=%u", g_animation_mode);
    }
}

static void ble_start_scan_for_ms(uint32_t duration_ms)
{
    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));

    disc_params.passive = 1;
    disc_params.itvl = 0x0030;
    disc_params.window = 0x0030;
    disc_params.filter_duplicates = 1;

    int32_t scan_duration = duration_ms > INT32_MAX ? INT32_MAX : static_cast<int32_t>(duration_ms);
    int rc = ble_gap_disc(own_addr_type, scan_duration, &disc_params, ble_gap_event, NULL);
    if (rc == 0) {
        ble_scanning = true;
        ESP_LOGI(TAG, "Follower scan started for %lu ms", static_cast<unsigned long>(duration_ms));
        return;
    }

    if (rc == BLE_HS_EALREADY) {
        return;
    }

    ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
}

static void ble_follower_scan_scheduler_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "Follower scan scheduler started");

    while (true) {
        if (!follower_synced) {
            ble_start_scan_for_ms(ORCHESTRA_STARTUP_WINDOW_MS);
            vTaskDelay(pdMS_TO_TICKS(ORCHESTRA_STARTUP_WINDOW_MS + 150));
            if (!follower_synced) {
                ESP_LOGW(TAG, "Follower startup scan ended without sync; retrying");
            }
            continue;
        }

        uint32_t now_ms = ble_get_sync_timestamp_ms();
        int64_t estimated_master_ms_signed = static_cast<int64_t>(now_ms) - follower_master_offset_ms;
        if (estimated_master_ms_signed < 0) {
            estimated_master_ms_signed = 0;
        }

        uint32_t estimated_master_ms = static_cast<uint32_t>(estimated_master_ms_signed);
        uint32_t until_on_ms = orchestra_ms_until_next_master_on_air(estimated_master_ms);
        if (until_on_ms > ORCHESTRA_SYNC_GUARD_MS) {
            vTaskDelay(pdMS_TO_TICKS(until_on_ms - ORCHESTRA_SYNC_GUARD_MS));
        }

        now_ms = ble_get_sync_timestamp_ms();
        estimated_master_ms_signed = static_cast<int64_t>(now_ms) - follower_master_offset_ms;
        if (estimated_master_ms_signed < 0) {
            estimated_master_ms_signed = 0;
        }
        estimated_master_ms = static_cast<uint32_t>(estimated_master_ms_signed);

        uint32_t on_remaining_ms = orchestra_master_on_air_remaining_ms(estimated_master_ms);
        uint32_t scan_ms = on_remaining_ms + ORCHESTRA_SYNC_GUARD_MS;
        if (scan_ms < 250) {
            scan_ms = 250;
        }

        ble_start_scan_for_ms(scan_ms);
        vTaskDelay(pdMS_TO_TICKS(scan_ms + 50));
    }
}

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

            if (ORCHESTRA_ROLE != OrchestraRole::Master) {
                return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
            }

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
        case BLE_GAP_EVENT_DISC: {
            struct ble_hs_adv_fields adv_fields;
            memset(&adv_fields, 0, sizeof(adv_fields));

            int rc = ble_hs_adv_parse_fields(&adv_fields,
                                             event->disc.data,
                                             event->disc.length_data);
            if (rc != 0 || !adv_fields.mfg_data || adv_fields.mfg_data_len == 0) {
                return 0;
            }

            OrchestraAdvPayload payload;
            if (!orchestra_decode_adv_payload(reinterpret_cast<const uint8_t *>(adv_fields.mfg_data),
                                              adv_fields.mfg_data_len,
                                              &payload)) {
                return 0;
            }

            if (ble_is_sync_source(payload)) {
                ble_apply_master_packet(payload);
            }
            return 0;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ble_scanning = false;
            ESP_LOGI(TAG, "Follower scan completed");
            return 0;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ble_connected = true;
                ble_advertising = false;
                ESP_LOGI(TAG, "BLE connected");
            } else {
                if (ORCHESTRA_ROLE == OrchestraRole::Master) {
                    ESP_LOGW(TAG, "BLE connect failed; restarting advertising");
                    ble_app_advertise();
                }
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ble_connected = false;
            ESP_LOGI(TAG, "BLE disconnected; reason=%d", event->disconnect.reason);
            if (ORCHESTRA_ROLE == OrchestraRole::Master) {
                ble_app_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_advertising = false;
            if (ORCHESTRA_ROLE != OrchestraRole::Master) {
                return 0;
            }

            if (ble_connected) {
                // A connection was made during the advertising window; stack stays up
                ESP_LOGI(TAG, "Advertising complete (connection active)");
            } else {
                if (!ble_startup_cycle_complete) {
                    ble_startup_cycle_complete = true;
                }
                // Normal timeout — shut down the stack completely for 38 s
                ESP_LOGI(TAG, "Advertising complete; tearing down BLE for 38 seconds");
                xTaskCreate(ble_deinit_and_restart_task, "BLE Restart", 4096, NULL, 5, NULL);
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
    if (ORCHESTRA_ROLE != OrchestraRole::Master) {
        return;
    }

    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    uint8_t orchestra_adv_data[ORCHESTRA_ADV_PAYLOAD_LEN] = {0};
    uint32_t sync_timestamp_ms = ble_get_sync_timestamp_ms();
    int adv_duration_ms = ble_startup_cycle_complete ? BLE_CYCLE_ADV_MS : BLE_STARTUP_ADV_MS;

    memset(&fields, 0, sizeof(fields));
    memset(&rsp_fields, 0, sizeof(rsp_fields));

    // Keep primary ADV packet compact (31-byte limit); place the full name in scan response.
    const char *name = ble_svc_gap_device_name();
    static const char short_name[] = "Pend-01";
    fields.name = (uint8_t *)short_name;
    fields.name_len = (uint8_t)strlen(short_name);
    fields.name_is_complete = 0;
    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = (uint8_t)strlen(name);
    rsp_fields.name_is_complete = 1;

    // General discoverable + BLE only
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    OrchestraAdvPayload payload = {
        .group_id = ORCHESTRA_GROUP_ID,
        .device_id = ORCHESTRA_DEVICE_ID,
        .role = ORCHESTRA_ROLE,
        .position = ORCHESTRA_POSITION,
        .master_id = ORCHESTRA_MASTER_ID,
        .sync_timestamp_ms = sync_timestamp_ms,
        .animation_mode = g_animation_mode,
    };

    size_t orchestra_adv_data_len = orchestra_encode_adv_payload(orchestra_adv_data,
                                                                 sizeof(orchestra_adv_data),
                                                                 payload);
    if (orchestra_adv_data_len != ORCHESTRA_ADV_PAYLOAD_LEN) {
        ESP_LOGE(TAG, "Failed to build orchestra advertising payload");
        return;
    }

    fields.mfg_data = orchestra_adv_data;
    fields.mfg_data_len = static_cast<uint8_t>(orchestra_adv_data_len);

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type,
                           NULL,
                           adv_duration_ms,
                           &adv_params,
                           ble_gap_event,
                           NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ble_advertising = true;
    ESP_LOGI(TAG,
             "Advertising started for %d ms (group=%u device=%u role=%s position=%d master=%u ts_ms=%lu anim=%u)",
             adv_duration_ms,
             ORCHESTRA_GROUP_ID,
             ORCHESTRA_DEVICE_ID,
             orchestra_role_to_string(ORCHESTRA_ROLE),
             ORCHESTRA_POSITION,
             ORCHESTRA_MASTER_ID,
             static_cast<unsigned long>(sync_timestamp_ms),
             g_animation_mode);
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

    if (ORCHESTRA_ROLE == OrchestraRole::Master) {
        ble_app_advertise();
        return;
    }

    if (!follower_scan_task_handle) {
        BaseType_t task_ok = xTaskCreate(ble_follower_scan_scheduler_task,
                                         "BLE Follow Scan",
                                         4096,
                                         NULL,
                                         5,
                                         &follower_scan_task_handle);
        if (task_ok != pdPASS) {
            follower_scan_task_handle = NULL;
            ESP_LOGE(TAG, "Failed to create follower scan scheduler task");
            return;
        }
    }

    ESP_LOGI(TAG, "Follower mode active; waiting for master sync");
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset; reason=%d", reason);
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    // Signal ble_deinit() that nimble_port_run() has returned before the task self-deletes
    if (ble_host_stopped_sem) {
        xSemaphoreGive(ble_host_stopped_sem);
    }
    nimble_port_freertos_deinit();
}

// Tears down the entire BLE stack. Must be called from a task that is NOT the NimBLE host task.
static void ble_deinit(void)
{
    if (!ble_initialized) {
        ESP_LOGW(TAG, "ble_deinit called but BLE not initialised");
        return;
    }

    ESP_LOGI(TAG, "Stopping NimBLE host");
    nimble_port_stop();

    // Wait for the host task to signal that nimble_port_run() has returned
    if (xSemaphoreTake(ble_host_stopped_sem, pdMS_TO_TICKS(3000)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout waiting for BLE host task to stop");
    }

    nimble_port_deinit();

    esp_err_t ret = esp_nimble_hci_and_controller_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_nimble_hci_and_controller_deinit failed: %d", ret);
    }

    ble_initialized = false;
    ble_advertising  = false;
    ESP_LOGI(TAG, "BLE stack torn down");
}

// Tears down BLE, waits 38 seconds, then re-initialises it so advertising resumes.
// Runs as a one-shot FreeRTOS task spawned from the ADV_COMPLETE event handler.
static void ble_deinit_and_restart_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "BLE off period: tearing down stack");
    ble_deinit();

    ESP_LOGI(TAG, "BLE sleeping for %d milliseconds", BLE_CYCLE_OFF_MS);
    vTaskDelay(pdMS_TO_TICKS(BLE_CYCLE_OFF_MS));

    ESP_LOGI(TAG, "Restarting BLE stack");
    ble_init();

    vTaskDelete(NULL);
}

void ble_init(void)
{
    if (ble_initialized) {
        ESP_LOGW(TAG, "ble_init called but BLE already initialised");
        return;
    }

    // Create the semaphore once; it is reused across deinit/reinit cycles
    if (!ble_host_stopped_sem) {
        ble_host_stopped_sem = xSemaphoreCreateBinary();
        if (!ble_host_stopped_sem) {
            ESP_LOGE(TAG, "Failed to create ble_host_stopped_sem");
            return;
        }
    }

    int rc;
    ESP_LOGI(TAG, "Initializing BLE");
    ESP_LOGI(TAG, "Initializing BLE controller");
    esp_nimble_hci_and_controller_init();
    ESP_LOGI(TAG, "BLE controller initialized");

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

    // Start NimBLE host task
    nimble_port_freertos_init(ble_host_task);

    ble_initialized = true;
    ESP_LOGI(TAG, "BLE init complete");
}

