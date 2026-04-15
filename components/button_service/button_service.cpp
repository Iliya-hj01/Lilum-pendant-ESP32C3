#include "button_service.h"
#include "led_engine.h"

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "BTN";

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
#define BUTTON_GPIO  GPIO_NUM_9
#define POLL_MS      5          // sample every 5 ms

// ---------------------------------------------------------------------------
// Timing thresholds
// ---------------------------------------------------------------------------
#define DEBOUNCE_MS         50
#define LONG_PRESS_MS       1000
#define DOUBLE_PRESS_GAP_MS 300   // max gap between two short presses for a double-press

// ---------------------------------------------------------------------------
// State saved before entering white mode so we can restore on exit
// ---------------------------------------------------------------------------
static bool   saved_manual_mode   = false;
static uint8_t saved_manual_pattern = 0;
static uint8_t saved_brightness     = 0;

// ---------------------------------------------------------------------------
// Brightness ramp state (runs while button is held)
// ---------------------------------------------------------------------------
typedef enum {
    RAMP_UP = 0,
    HOLD_MAX,
    HOLD_MIN,
} RampPhase;

static RampPhase rampPhase   = RAMP_UP;
static uint32_t  rampPhaseT0 = 0;   // timestamp when current phase started
static uint32_t  rampStepT   = 0;   // timestamp of last brightness step

// ---------------------------------------------------------------------------
// Button-press state machine
// ---------------------------------------------------------------------------
static bool     rawLast      = true;   // last raw reading (true = released)
static bool     stable       = true;   // debounced state
static uint32_t tChanged     = 0;      // when raw last changed

static bool     pressActive  = false;  // a stable press is in progress
static uint32_t tPressStart  = 0;      // when the current press started
static bool     longUsed     = false;  // long-press action already triggered

// For double-press detection
static bool     waitingSecondPress  = false;  // got one short-press release, waiting for another
static uint32_t tFirstRelease       = 0;      // when the first short press was released
static bool     whiteWasActiveBeforePress = false; // track if white mode was on when press started

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void enter_white_mode(void)
{
    saved_manual_mode    = g_manual_mode_active;
    saved_manual_pattern = g_manual_pattern;
    saved_brightness     = workingBrightness;
    led_engine_set_white_mode(true);
    ESP_LOGI(TAG, "White mode ON");
}

static void exit_white_mode(void)
{
    led_engine_set_white_mode(false);
    g_manual_mode_active = saved_manual_mode;
    g_manual_pattern     = saved_manual_pattern;
    workingBrightness    = saved_brightness;
    ESP_LOGI(TAG, "White mode OFF — restored previous mode");
}

static void handle_single_short_press(void)
{
    if (led_engine_is_white_mode()) {
        // Exit white mode and restore previous animation state
        exit_white_mode();
        return;
    }

    // First short press switches from auto → manual (pattern 0).
    // Subsequent presses advance through manual patterns.
    if (!g_manual_mode_active) {
        g_manual_mode_active = true;
        g_manual_pattern = 0;
        ESP_LOGI(TAG, "Switched to manual mode, pattern 0");
    } else {
        g_manual_pattern = (g_manual_pattern + 1) % led_engine_get_num_manual_patterns();
        ESP_LOGI(TAG, "Manual pattern -> %u", g_manual_pattern);
    }
}

static void handle_double_press(void)
{
    if (led_engine_is_white_mode()) {
        exit_white_mode();
    } else {
        enter_white_mode();
    }
}

// ---------------------------------------------------------------------------
// Brightness ramp (called continuously while button is held ≥ LONG_PRESS_MS)
// ---------------------------------------------------------------------------
static void brightness_ramp_tick(uint32_t now)
{
    if ((now - rampStepT) < LED_BRIGHTNESS_STEP_MS) {
        return;
    }
    rampStepT = now;

    switch (rampPhase) {
    case RAMP_UP:
        if (workingBrightness < LED_BRIGHTNESS_MAX) {
            workingBrightness++;
        }
        if (workingBrightness >= LED_BRIGHTNESS_MAX) {
            workingBrightness = LED_BRIGHTNESS_MAX;
            rampPhase   = HOLD_MAX;
            rampPhaseT0 = now;
        }
        break;

    case HOLD_MAX:
        workingBrightness = LED_BRIGHTNESS_MAX;
        if ((now - rampPhaseT0) >= LED_BRIGHTNESS_HOLD_MS) {
            workingBrightness = LED_BRIGHTNESS_MIN;
            rampPhase   = HOLD_MIN;
            rampPhaseT0 = now;
        }
        break;

    case HOLD_MIN:
        workingBrightness = LED_BRIGHTNESS_MIN;
        if ((now - rampPhaseT0) >= LED_BRIGHTNESS_HOLD_MS) {
            rampPhase = RAMP_UP;
        }
        break;
    }
}

// ---------------------------------------------------------------------------
// Main polling function (called every POLL_MS from the task)
// ---------------------------------------------------------------------------
static void button_poll(void)
{
    bool raw = gpio_get_level(BUTTON_GPIO);  // true = released (pull-up)
    uint32_t now = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;

    // --- Track raw edges for debounce ---
    if (raw != rawLast) {
        rawLast  = raw;
        tChanged = now;
    }

    // --- Debounce ---
    if ((now - tChanged) >= DEBOUNCE_MS && stable != raw) {
        stable = raw;

        if (!stable) {
            // ---- Stable PRESS edge (HIGH → LOW) ----
            pressActive = true;
            longUsed    = false;
            tPressStart = now;

            // Reset ramp state
            rampPhase   = RAMP_UP;
            rampPhaseT0 = 0;
            rampStepT   = now;

            // Remember if white mode was already on at the start of this press
            whiteWasActiveBeforePress = led_engine_is_white_mode();

        } else {
            // ---- Stable RELEASE edge (LOW → HIGH) ----
            if (pressActive) {
                uint32_t duration = now - tPressStart;

                if (!longUsed && duration >= DEBOUNCE_MS && duration < LONG_PRESS_MS) {
                    // Short press detected
                    if (waitingSecondPress && (now - tFirstRelease) <= DOUBLE_PRESS_GAP_MS) {
                        // Second press of a double-press
                        waitingSecondPress = false;
                        handle_double_press();
                    } else {
                        // Could be first of a double-press — wait to see
                        waitingSecondPress = true;
                        tFirstRelease      = now;
                    }
                }
                // If long press was used, nothing extra happens on release.

                pressActive = false;
            }
        }
    }

    // --- Double-press timeout: no second press arrived → treat as single short press ---
    if (waitingSecondPress && (now - tFirstRelease) > DOUBLE_PRESS_GAP_MS) {
        waitingSecondPress = false;
        handle_single_short_press();
    }

    // --- Long-press brightness ramp while held ---
    if (pressActive && !stable) {
        uint32_t held = now - tPressStart;
        if (held >= LONG_PRESS_MS) {
            longUsed = true;
            brightness_ramp_tick(now);
        }
    }
}

// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------
static void button_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "Button task started (GPIO%d, poll %d ms)", BUTTON_GPIO, POLL_MS);

    TickType_t xLastWake = xTaskGetTickCount();
    while (true) {
        button_poll();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(POLL_MS));
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void button_service_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << BUTTON_GPIO;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    xTaskCreate(button_task, "Button", 2048, NULL, 2, NULL);
}
