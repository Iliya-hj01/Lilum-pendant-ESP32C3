#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise I2S peripheral and start the microphone sampling task.
 * Call once from app_main() after initArduino().
 */
void mic_service_init(void);

/**
 * Current volume level (0-255, smoothed).
 */
uint8_t mic_get_volume(void);

/**
 * Returns true once per detected beat, then auto-clears.
 * Poll this from your print/LED task.
 */
bool mic_get_beat(void);

/**
 * Estimated tempo in beats per minute (0 if not enough data yet).
 */
uint16_t mic_get_bpm(void);

#ifdef __cplusplus
}
#endif
