

// Forward declaration for use in timer callback
void orchestra_ble_deinit(void);

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
#include "freertos/timers.h"

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
#include "led_engine.h"

static const char *TAG = "BLE";

void ble_set_logging(bool enabled) {
    esp_log_level_set(TAG, enabled ? ESP_LOG_INFO : ESP_LOG_NONE);
}

static constexpr uint32_t BLE_STARTUP_ADV_MS = ORCHESTRA_STARTUP_WINDOW_MS;
static constexpr uint32_t BLE_CYCLE_OFF_MS = ORCHESTRA_CYCLE_OFF_MS;
static constexpr uint32_t BLE_CYCLE_ADV_MS = ORCHESTRA_CYCLE_ADV_MS;
static bool ble_initialized = false;
static bool ble_advertising = false;
static bool ble_scanning = false;
static bool ble_connected = false;
static bool ble_startup_cycle_complete = false;
static bool follower_synced = false;
static bool follower_ever_synced = false;  // True once first sync is achieved.
static int64_t follower_master_offset_ms = 0;  // Local time minus master's timestamp.
static TaskHandle_t follower_scan_task_handle = NULL;
static TaskHandle_t master_adv_refresh_task_handle = NULL;
static uint32_t scan_adv_count = 0;  // Count of advertisements seen in current scan
static uint8_t last_advertised_mode = 0;  // Track mode currently in the adv payload
static bool master_heard_in_scan = false;  // Set when master packet received during a scan
static uint8_t consecutive_missed_scans = 0;  // Count of consecutive scan windows without master
static uint32_t last_master_heard_local_ms = 0;  // Follower local time when master was last received

// GATT inactivity timer (master only)
#define BLE_GATT_INACTIVITY_TIMEOUT_MS 300000UL
static TimerHandle_t gatt_inactivity_timer = NULL;

static void gatt_inactivity_timeout_cb(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "GATT inactivity timeout reached, disconnecting and tearing down BLE stack");
    if (ble_connected) {
        ble_gap_terminate(0, BLE_ERR_REM_USER_CONN_TERM); // 0 = any connection
    }
    orchestra_ble_deinit();
}

static void gatt_inactivity_timer_reset() {
    if (gatt_inactivity_timer) {
        xTimerStop(gatt_inactivity_timer, 0);
        xTimerChangePeriod(gatt_inactivity_timer, pdMS_TO_TICKS(BLE_GATT_INACTIVITY_TIMEOUT_MS), 0);
        xTimerStart(gatt_inactivity_timer, 0);
    }
}

// Advertising timeout timer – enforces the startup window independently of NimBLE,
// because ble_gap_adv_set_fields() (called by the payload-refresh task) can
// internally restart advertising and reset the NimBLE duration counter.
static TimerHandle_t adv_timeout_timer = NULL;

static void adv_timeout_cb(TimerHandle_t xTimer) {
    (void)xTimer;
    ESP_LOGI(TAG, "Advertising timeout timer fired");
    ble_gap_adv_stop();
    ble_advertising = false;
    if (!ble_connected) {
        ESP_LOGI(TAG, "No connection during advertising window; tearing down BLE permanently");
        orchestra_ble_deinit();
    }
}

bool ble_is_follower_synced(void)
{
    return follower_synced;
}

uint32_t ble_ms_until_next_scan(void)
{
    // Follower scans continuously; always return 0.
    return 0;
}

bool ble_is_currently_advertising(void)
{
    return ble_advertising;
}

bool ble_is_currently_scanning(void)
{
    return ble_scanning;
}

// Animation mode is owned by sketch.cpp and shared with BLE.
extern uint8_t g_animation_mode;

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
static void ble_deinit_and_restart_task(void *param); // One-shot task that deinitializes BLE, waits, then reinitializes BLE.
static uint32_t ble_get_sync_timestamp_ms(void); // Monotonic timestamp used as the sync reference in advertising.
static bool ble_is_sync_source(const OrchestraAdvPayload &payload); // Checks whether an advertisement came from the configured master.
static void ble_apply_master_packet(const OrchestraAdvPayload &payload); // Updates follower sync and animation based on master packet.
static void ble_follower_scan_scheduler_task(void *param); // Starts continuous BLE scanning for the follower.
static void ble_start_continuous_scan(void); // Starts a continuous passive BLE scan (BLE_HS_FOREVER).
static void ble_master_adv_refresh_task(void *param); // Refreshes adv payload when animation mode changes.
static void ble_update_adv_payload(void); // Updates adv data in-place with current mode and timestamp.

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .includes = NULL,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &chr_uuid.u,
                .access_cb = mode_chr_access,
                .arg = NULL,
                .descriptors = NULL,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .min_key_size = 0,
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
    const bool device_ok = payload.device_id == ORCHESTRA_MASTER_ID;
    return role_ok && group_ok && device_ok;
}

static void ble_apply_master_packet(const OrchestraAdvPayload &payload)
{
    const uint32_t now_ms = ble_get_sync_timestamp_ms();
    follower_master_offset_ms = static_cast<int64_t>(now_ms) - static_cast<int64_t>(payload.sync_timestamp_ms);

    if (!follower_synced) {
        follower_synced = true;
        follower_ever_synced = true;
        ESP_LOGI(TAG,
                 "Follower synced to master device=%u group=%u master_ts=%lu local_ts=%lu offset=%lld",
                 payload.device_id,
                 payload.group_id,
                 static_cast<unsigned long>(payload.sync_timestamp_ms),
                 static_cast<unsigned long>(now_ms),
                 static_cast<long long>(follower_master_offset_ms));
    }

    master_heard_in_scan = true;
    last_master_heard_local_ms = now_ms;

    if (g_animation_mode != payload.animation_mode) {
        g_animation_mode = payload.animation_mode;
        ESP_LOGI(TAG, "Animation updated by master to mode=%u", g_animation_mode);
    }
}

// Starts a continuous passive scan (BLE_HS_FOREVER).
// Called once at startup and re-called from BLE_GAP_EVENT_DISC_COMPLETE
// if the scan ends for any reason.
static void ble_start_continuous_scan(void)
{
    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));

    disc_params.passive = 1;
    disc_params.itvl = 0x0030;
    disc_params.window = 0x0030;
    disc_params.filter_duplicates = 0;

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
    if (rc == 0) {
        ble_scanning = true;
        ESP_LOGI(TAG, "Continuous follower scan ACTIVE");
        return;
    }
    if (rc == BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Scan already active (BLE_HS_EALREADY)");
        return;
    }
    ESP_LOGE(TAG, "ble_gap_disc FAILED: rc=%d", rc);
}

static void ble_follower_scan_scheduler_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, ">>> FOLLOWER CONTINUOUS SCAN TASK STARTED <<<");
    vTaskDelay(pdMS_TO_TICKS(100));  // Let BLE host fully initialize

    ble_start_continuous_scan();

    // Task stays alive so the handle remains valid, but has nothing else to do.
    // The scan restarts from BLE_GAP_EVENT_DISC_COMPLETE if it ever stops.
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
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

    // Reset inactivity timer on any GATT access
    gatt_inactivity_timer_reset();
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

            // Count all received advertisements
            scan_adv_count++;
            // Log non-orchestra advertisements at DEBUG level only
            ESP_LOGD(TAG, "ADV RX (#%lu): addr=%02x:%02x:%02x:%02x:%02x:%02x rssi=%d len=%u",
                     static_cast<unsigned long>(scan_adv_count),
                     event->disc.addr.val[0], event->disc.addr.val[1],
                     event->disc.addr.val[2], event->disc.addr.val[3],
                     event->disc.addr.val[4], event->disc.addr.val[5],
                     event->disc.rssi, event->disc.length_data);

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
                if (!master_heard_in_scan) {
                    ESP_LOGI(TAG, "Orchestra ADV (sync): dev=%u ts=%lu anim=%u",
                             payload.device_id,
                             static_cast<unsigned long>(payload.sync_timestamp_ms),
                             payload.animation_mode);
                }
                ble_apply_master_packet(payload);
            } else {
                ESP_LOGD(TAG, "Orchestra ADV (other): role=%u group=%u dev=%u master=%u",
                         static_cast<uint8_t>(payload.role), payload.group_id,
                         payload.device_id, payload.master_id);
            }
            return 0;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ble_scanning = false;
            ESP_LOGI(TAG, "Scan completed (adverts RX: %lu) — restarting continuous scan",
                     static_cast<unsigned long>(scan_adv_count));
            master_heard_in_scan = false;
            scan_adv_count = 0;
            // Immediately restart scanning so the follower never stops listening
            if (ORCHESTRA_ROLE == OrchestraRole::Follower) {
                ble_start_continuous_scan();
            }
            return 0;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ble_connected = true;
                ble_advertising = false;
                ESP_LOGI(TAG, "BLE connected");
                // Cancel advertising timeout – device is connected now
                if (adv_timeout_timer) {
                    xTimerStop(adv_timeout_timer, 0);
                }
                gatt_inactivity_timer_reset();
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
            if (gatt_inactivity_timer) {
                xTimerStop(gatt_inactivity_timer, 0);
            }
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
                // No connection was established during the advertising window.
                // Tear down the BLE stack permanently to save power.
                ESP_LOGI(TAG, "Advertising complete with no connection; tearing down BLE permanently");
                orchestra_ble_deinit();
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

// Updates the advertising payload with the current animation mode and timestamp
// while the master is already advertising. Safe to call at any time.
static void ble_update_adv_payload(void)
{
    if (!ble_initialized || !ble_advertising) {
        return;
    }

    struct ble_hs_adv_fields fields;
    uint8_t orchestra_adv_data[ORCHESTRA_ADV_PAYLOAD_LEN] = {0};
    uint32_t sync_timestamp_ms = ble_get_sync_timestamp_ms();

    memset(&fields, 0, sizeof(fields));

    static const char short_name[] = "Pend-01";
    fields.name = (uint8_t *)short_name;
    fields.name_len = (uint8_t)strlen(short_name);
    fields.name_is_complete = 0;
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

    size_t len = orchestra_encode_adv_payload(orchestra_adv_data, sizeof(orchestra_adv_data), payload);
    if (len != ORCHESTRA_ADV_PAYLOAD_LEN) {
        return;
    }

    fields.mfg_data = orchestra_adv_data;
    fields.mfg_data_len = static_cast<uint8_t>(len);

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv payload refresh failed: %d", rc);
        return;
    }

    last_advertised_mode = g_animation_mode;
    ESP_LOGI(TAG, "Adv payload refreshed: anim=%u ts=%lu", g_animation_mode,
             static_cast<unsigned long>(sync_timestamp_ms));
}

// Runs while the master is advertising; checks if animation mode changed
// and refreshes the adv payload if so.
static void ble_master_adv_refresh_task(void *param)
{
    (void)param;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (!ble_initialized || !ble_advertising) {
            continue;
        }
        // Always refresh so the timestamp stays current for followers.
        ble_update_adv_payload();
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
    // Use BLE_HS_FOREVER for NimBLE because the payload-refresh task calls
    // ble_gap_adv_set_fields() every 500 ms which resets the NimBLE duration
    // counter.  The actual timeout is enforced by adv_timeout_timer instead.
    int nimble_duration = BLE_HS_FOREVER;

    // Cycle animation mode at the start of each post-startup advertising window
    // so the mode change is always visible to followers.
    if (ble_startup_cycle_complete) {
        g_animation_mode = (g_animation_mode + 1) % led_engine_get_num_auto_patterns();
        ESP_LOGI(TAG, "Master animation mode cycled to %u", g_animation_mode);
    }

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
                           nimble_duration,
                           &adv_params,
                           ble_gap_event,
                           NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ble_advertising = true;
    last_advertised_mode = g_animation_mode;

    // Start (or restart) the software timer that enforces the advertising window
    if (adv_timeout_timer) {
        xTimerStop(adv_timeout_timer, 0);
        xTimerChangePeriod(adv_timeout_timer, pdMS_TO_TICKS(adv_duration_ms), 0);
        xTimerStart(adv_timeout_timer, 0);
    }

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
    ESP_LOGI(TAG, ">>> BLE HOST SYNCHRONIZED <<<");
    
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc == 0) {
        // Create GATT inactivity timer (master only)
        if (ORCHESTRA_ROLE == OrchestraRole::Master && !gatt_inactivity_timer) {
            gatt_inactivity_timer = xTimerCreate("GATT Inactivity", pdMS_TO_TICKS(BLE_GATT_INACTIVITY_TIMEOUT_MS), pdFALSE, NULL, gatt_inactivity_timeout_cb);
        }
        // Create advertising timeout timer (master only)
        if (ORCHESTRA_ROLE == OrchestraRole::Master && !adv_timeout_timer) {
            adv_timeout_timer = xTimerCreate("Adv Timeout", pdMS_TO_TICKS(BLE_STARTUP_ADV_MS), pdFALSE, NULL, adv_timeout_cb);
        }
        ESP_LOGI(TAG,
                 "BLE address: %02X:%02X:%02X:%02X:%02X:%02X",
                 addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0]);
    }

    if (ORCHESTRA_ROLE == OrchestraRole::Master) {
        ble_app_advertise();
        if (!master_adv_refresh_task_handle) {
            xTaskCreate(ble_master_adv_refresh_task, "BLE AdvRefresh", 3072, NULL, 4, &master_adv_refresh_task_handle);
        }
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
void orchestra_ble_deinit(void)
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
    ble_advertising = false;
    ESP_LOGI(TAG, "BLE stack torn down");
}

// Tears down BLE, waits 38 seconds, then re-initialises it so advertising resumes.
// Runs as a one-shot FreeRTOS task spawned from the ADV_COMPLETE event handler.
static void ble_deinit_and_restart_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "BLE off period: tearing down stack");
    orchestra_ble_deinit();

    ESP_LOGI(TAG, "BLE sleeping for %d milliseconds", BLE_CYCLE_OFF_MS);
    vTaskDelay(pdMS_TO_TICKS(BLE_CYCLE_OFF_MS));

    ESP_LOGI(TAG, "Restarting BLE stack");
    ble_init();

    vTaskDelete(NULL);
}

void ble_init(void)
{
    ESP_LOGI(TAG, ">>> ble_init() CALLED <<<");
    
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

