#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise the I2C bus and LSM6DS3TR-C IMU, then start the
 * sampling task.  Call once from app_main() after initArduino().
 */
void imu_service_init(void);

/**
 * Per-axis acceleration in milli-g (smoothed).
 */
void imu_get_accel_mg(float *ax, float *ay, float *az);

/**
 * Returns true exactly once per double-dip detection.
 * Subsequent calls return false until the next gesture.
 */
bool imu_gesture_double_dip(void);

/**
 * Current gesture detector state as a short string (for tuning/debug).
 */
const char *imu_gesture_state_str(void);

/**
 * Enable or disable gesture recognition at runtime.
 * When disabled the state machine resets to IDLE.
 */
void imu_gesture_set_enabled(bool enable);
bool imu_gesture_is_enabled(void);

#ifdef __cplusplus
}
#endif
