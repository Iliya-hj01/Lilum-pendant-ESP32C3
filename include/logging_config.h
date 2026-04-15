#pragma once

// ── Print-task logging configuration ───────────────────────
// Adjust these values to control what appears in the serial
// monitor and how often.

// Print interval in milliseconds.
// Examples: 1000 = 1 s, 500 = 0.5 s, 100 = 0.1 s
static constexpr uint32_t LOG_INTERVAL_MS = 1000;

// Enable / disable individual sections of the print line.
static constexpr bool LOG_ENABLE_MAIN = false;   // Role & animation
static constexpr bool LOG_ENABLE_BLE = false;
static constexpr bool LOG_ENABLE_MIC = false;
static constexpr bool LOG_ENABLE_IMU = false;
static constexpr bool LOG_ENABLE_GESTURE = false;  // Gesture state (for tuning)
static constexpr bool LOG_ENABLE_BATTERY = true;  // Battery/temp status

// Enable verbose BLE stack logs (NimBLE, BLE_INIT tags).
static constexpr bool LOG_ENABLE_BLE_STACK = false;
