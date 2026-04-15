#include "orchestra_payload.h"

// Serializes the fixed-width manufacturer payload into a compact 17-byte buffer.
size_t orchestra_encode_adv_payload(uint8_t *buffer, size_t buffer_len, const OrchestraAdvPayload &payload)
{
    if (!buffer || buffer_len < ORCHESTRA_ADV_PAYLOAD_LEN) {
        return 0;
    }

    // Fields are stored little-endian so encoder and decoder stay symmetric.
    buffer[0] = static_cast<uint8_t>(ORCHESTRA_ADV_COMPANY_ID & 0xFF);
    buffer[1] = static_cast<uint8_t>((ORCHESTRA_ADV_COMPANY_ID >> 8) & 0xFF);
    buffer[2] = ORCHESTRA_ADV_DATA_VERSION;
    buffer[3] = static_cast<uint8_t>(payload.group_id & 0xFF);
    buffer[4] = static_cast<uint8_t>((payload.group_id >> 8) & 0xFF);
    buffer[5] = static_cast<uint8_t>(payload.device_id & 0xFF);
    buffer[6] = static_cast<uint8_t>((payload.device_id >> 8) & 0xFF);
    buffer[7] = static_cast<uint8_t>(payload.role);
    buffer[8] = static_cast<uint8_t>(static_cast<uint16_t>(payload.position) & 0xFF);
    buffer[9] = static_cast<uint8_t>((static_cast<uint16_t>(payload.position) >> 8) & 0xFF);
    buffer[10] = static_cast<uint8_t>(payload.master_id & 0xFF);
    buffer[11] = static_cast<uint8_t>((payload.master_id >> 8) & 0xFF);
    buffer[12] = static_cast<uint8_t>(payload.sync_timestamp_ms & 0xFF);
    buffer[13] = static_cast<uint8_t>((payload.sync_timestamp_ms >> 8) & 0xFF);
    buffer[14] = static_cast<uint8_t>((payload.sync_timestamp_ms >> 16) & 0xFF);
    buffer[15] = static_cast<uint8_t>((payload.sync_timestamp_ms >> 24) & 0xFF);
    buffer[16] = payload.animation_mode;

    return ORCHESTRA_ADV_PAYLOAD_LEN;
}

// Decodes and validates a manufacturer payload before exposing typed fields.
bool orchestra_decode_adv_payload(const uint8_t *buffer, size_t buffer_len, OrchestraAdvPayload *out_payload)
{
    if (!buffer || !out_payload || buffer_len < ORCHESTRA_ADV_PAYLOAD_LEN) {
        return false;
    }

    // Reject unrelated advertisements early using the company identifier and version.
    const uint16_t company_id = static_cast<uint16_t>(buffer[0]) |
                                (static_cast<uint16_t>(buffer[1]) << 8);
    if (company_id != ORCHESTRA_ADV_COMPANY_ID) {
        return false;
    }

    if (buffer[2] != ORCHESTRA_ADV_DATA_VERSION) {
        return false;
    }

    out_payload->group_id = static_cast<uint16_t>(buffer[3]) |
                            (static_cast<uint16_t>(buffer[4]) << 8);
    out_payload->device_id = static_cast<uint16_t>(buffer[5]) |
                             (static_cast<uint16_t>(buffer[6]) << 8);
    out_payload->role = static_cast<OrchestraRole>(buffer[7]);
    out_payload->position = static_cast<int16_t>(
        static_cast<uint16_t>(buffer[8]) |
        (static_cast<uint16_t>(buffer[9]) << 8));
    out_payload->master_id = static_cast<uint16_t>(buffer[10]) |
                             (static_cast<uint16_t>(buffer[11]) << 8);
    out_payload->sync_timestamp_ms = static_cast<uint32_t>(buffer[12]) |
                                     (static_cast<uint32_t>(buffer[13]) << 8) |
                                     (static_cast<uint32_t>(buffer[14]) << 16) |
                                     (static_cast<uint32_t>(buffer[15]) << 24);
    out_payload->animation_mode = buffer[16];

    return true;
}

// Startup is treated as continuously on-air; after that the master alternates between
// off periods and short advertising windows (off first, then on).
bool orchestra_is_master_on_air(uint32_t master_time_ms)
{
    if (master_time_ms < ORCHESTRA_STARTUP_WINDOW_MS) {
        return true;
    }

    const uint32_t periodic_ms = ORCHESTRA_CYCLE_ADV_MS + ORCHESTRA_CYCLE_OFF_MS;
    const uint32_t cycle_phase_ms = (master_time_ms - ORCHESTRA_STARTUP_WINDOW_MS) % periodic_ms;
    return cycle_phase_ms >= ORCHESTRA_CYCLE_OFF_MS;
}

// Returns zero when already inside an advertising window, otherwise the delay until the
// next on-air slot begins in the repeating advertise/off cycle.
uint32_t orchestra_ms_until_next_master_on_air(uint32_t master_time_ms)
{
    if (orchestra_is_master_on_air(master_time_ms)) {
        return 0;
    }

    if (master_time_ms < ORCHESTRA_STARTUP_WINDOW_MS) {
        return 0;
    }

    const uint32_t periodic_ms = ORCHESTRA_CYCLE_ADV_MS + ORCHESTRA_CYCLE_OFF_MS;
    const uint32_t cycle_phase_ms = (master_time_ms - ORCHESTRA_STARTUP_WINDOW_MS) % periodic_ms;
    return ORCHESTRA_CYCLE_OFF_MS - cycle_phase_ms;
}

// Reports how much of the current advertising slot is left so followers can size their
// scan window to cover the master's remaining transmission time.
uint32_t orchestra_master_on_air_remaining_ms(uint32_t master_time_ms)
{
    if (!orchestra_is_master_on_air(master_time_ms)) {
        return 0;
    }

    if (master_time_ms < ORCHESTRA_STARTUP_WINDOW_MS) {
        return ORCHESTRA_STARTUP_WINDOW_MS - master_time_ms;
    }

    const uint32_t periodic_ms = ORCHESTRA_CYCLE_ADV_MS + ORCHESTRA_CYCLE_OFF_MS;
    const uint32_t cycle_phase_ms = (master_time_ms - ORCHESTRA_STARTUP_WINDOW_MS) % periodic_ms;
    return periodic_ms - cycle_phase_ms;
}
