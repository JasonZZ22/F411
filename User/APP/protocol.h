#ifndef DART_PROTOCOL_H
#define DART_PROTOCOL_H
#include <stdint.h>

/* Wire format: AA 01 dxH dxL dyH dyL rollH rollL checksum.
 * Values are signed big-endian int16, normalized error * 1000. */
#define TARGET_LOST_TIMEOUT_MS 150u

typedef struct {
    int16_t dx_raw, dy_raw, roll_raw;
    float dx, dy, roll;
    uint32_t last_ms, seq;
    uint8_t valid;
} target_state_t;

typedef struct {
    uint32_t rx_bytes, pkt_ok, pkt_checksum_error, pkt_range_error;
    uint32_t timeout_count, dma_restart_count, uart_error_count;
} target_debug_t;

extern volatile target_state_t g_target;
extern volatile target_debug_t g_target_debug;
void protocol_init(void);
void protocol_feed(uint8_t byte);
void protocol_update_timeout(void);
void protocol_copy_target(target_state_t *out);
#endif
