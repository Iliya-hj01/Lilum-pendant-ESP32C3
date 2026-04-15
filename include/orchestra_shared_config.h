#pragma once

#include <stdint.h>

enum class OrchestraRole : uint8_t {
    Follower = 0,
    Master = 1,
};

// Orchestra node configuration.
// Update these values per device before flashing. In a future revision these
// will be managed from a phone app and persisted in non-volatile storage.
static constexpr uint16_t ORCHESTRA_GROUP_ID = 1;
static constexpr uint16_t ORCHESTRA_DEVICE_ID = 2;
static constexpr OrchestraRole ORCHESTRA_ROLE = OrchestraRole::Master;
static constexpr int16_t ORCHESTRA_POSITION = -1;  // Undefined until position mapping is finalized.
static constexpr uint16_t ORCHESTRA_MASTER_ID = 2;

// BLE timing plan (milliseconds).
// Master: advertise 10 s on startup, then 2 s on / 10 s off.
// Follower: scan with the same timing plan, aligned to master's timestamp after sync.
static constexpr uint32_t ORCHESTRA_STARTUP_WINDOW_MS = 60000;
static constexpr uint32_t ORCHESTRA_CYCLE_ADV_MS = 2000;
static constexpr uint32_t ORCHESTRA_CYCLE_OFF_MS = 10000;
static constexpr uint32_t ORCHESTRA_SYNC_GUARD_MS = 500;

// Recovery behavior.
static constexpr bool ORCHESTRA_RESCAN_ON_SYNC_LOSS = false;  // If true, start a long scan when sync is lost; if false, stop scanning.
static constexpr uint8_t ORCHESTRA_MAX_MISSED_SCANS = 3;      // Consecutive missed scan windows before declaring sync lost.

// Shared behavior defaults.
static constexpr uint8_t ORCHESTRA_DEFAULT_ANIMATION_MODE = 0;

static inline const char *orchestra_role_to_string(OrchestraRole role)
{
    return role == OrchestraRole::Master ? "master" : "follower";
}