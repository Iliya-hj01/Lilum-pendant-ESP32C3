#pragma once

#include <stddef.h>
#include <stdint.h>

#include "orchestra_config.h"

struct OrchestraAdvPayload {
    uint16_t group_id;
    uint16_t device_id;
    OrchestraRole role;
    int16_t position;
    uint16_t master_id;
    uint32_t sync_timestamp_ms;
    uint8_t animation_mode;
};

static constexpr size_t ORCHESTRA_ADV_PAYLOAD_LEN = 17;

size_t orchestra_encode_adv_payload(uint8_t *buffer, size_t buffer_len, const OrchestraAdvPayload &payload);
bool orchestra_decode_adv_payload(const uint8_t *buffer, size_t buffer_len, OrchestraAdvPayload *out_payload);

bool orchestra_is_master_on_air(uint32_t master_time_ms);
uint32_t orchestra_ms_until_next_master_on_air(uint32_t master_time_ms);
uint32_t orchestra_master_on_air_remaining_ms(uint32_t master_time_ms);
