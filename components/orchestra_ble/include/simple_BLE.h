#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void ble_init(void);
void ble_set_logging(bool enabled);
bool ble_is_follower_synced(void);
uint32_t ble_ms_until_next_scan(void);
bool ble_is_currently_advertising(void);
bool ble_is_currently_scanning(void);

#ifdef __cplusplus
}
#endif
