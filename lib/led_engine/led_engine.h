#pragma once

#include <stdint.h>
#include <stdbool.h>

// Brightness limits
#define LED_BRIGHTNESS_MIN   40
#define LED_BRIGHTNESS_MAX   200
#define LED_BRIGHTNESS_WHITE 120   // brightness used in white-light mode
#define LED_BRIGHTNESS_HOLD_MS 300 // hold duration at min/max during long-press ramp
#define LED_BRIGHTNESS_STEP_MS 30  // interval between brightness increments

#ifdef __cplusplus
extern "C" {
#endif

void led_engine_setup(void);
void led_engine_loop(void);
uint8_t led_engine_get_num_auto_patterns(void);
uint8_t led_engine_get_num_manual_patterns(void);

/// Enter solid-white mode (overrides auto/manual animations).
void led_engine_set_white_mode(bool enable);

/// Returns true when white mode is active.
bool led_engine_is_white_mode(void);

#ifdef __cplusplus
}
#endif

extern uint8_t g_animation_mode;       // current auto-pattern index (also set by BLE)
extern bool    g_manual_mode_active;   // true = manual single-pattern mode
extern uint8_t g_manual_pattern;       // current manual-pattern index
extern bool    g_output_state;         // GPIO 20 output state (true = HIGH)
extern uint8_t workingBrightness;      // current LED brightness
