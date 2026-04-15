#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BAT_CHG_NOT_CHARGING,      // no charger connected / discharging
    BAT_CHG_CHARGING_OK,       // 1. charging properly
    BAT_CHG_TEMP_INHIBITED,    // 2. charging stopped due to temperature
    BAT_CHG_COMPLETE,          // 3. charging finished (battery full)
    BAT_CHG_TEMP_CRITICAL,     // 4. not charging AND temp out of range
} battery_charge_status_t;

/**
 * Initialise the battery service (IP5306 I2C + NTC ADC on GPIO0).
 * Must be called AFTER imu_service_init() so the I2C bus is already up.
 */
void battery_service_init(void);

/** Battery level in approximate percent (0, 25, 50, 75, 100) or -1 on error. */
int8_t battery_get_level(void);

/** True while the battery is being charged. */
bool battery_is_charging(void);

/** True when the battery is fully charged. */
bool battery_is_full(void);

/** NTC temperature in degrees Celsius (NaN on ADC error). */
float battery_get_temperature(void);

/** True if the NTC temperature is within the allowed operating range (0–45 °C). */
bool battery_temp_in_range(void);

/** Current charging status (for LED indication, etc.). */
battery_charge_status_t battery_get_charge_status(void);

/**
 * Returns true exactly once after a short key-press on the IP5306
 * power button is detected.  Subsequent calls return false until the
 * next press event.
 */
bool battery_key_short_press(void);

/** Returns true exactly once after a long key-press is detected. */
bool battery_key_long_press(void);

/** Returns true exactly once after a double-click is detected. */
bool battery_key_double_click(void);

#ifdef __cplusplus
}
#endif
