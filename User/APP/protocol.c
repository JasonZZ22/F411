#include "protocol.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

volatile target_state_t g_target;
volatile target_debug_t g_target_debug;
static uint8_t s_frame[9];
static uint8_t s_count;

static int16_t signed_be16(uint8_t hi, uint8_t lo)
{
    uint16_t bits = ((uint16_t)hi << 8) | lo;
    /* Avoid implementation-defined unsigned-to-signed conversion. */
    return bits < 0x8000u ? (int16_t)bits : (int16_t)((int32_t)bits - 65536);
}

void protocol_init(void)
{
    s_count = 0;
    g_target.dx_raw = g_target.dy_raw = g_target.roll_raw = 0;
    g_target.dx = g_target.dy = g_target.roll = 0.0f;
    g_target.last_ms = g_target.seq = 0;
    g_target.valid = 0;
    g_target_debug.rx_bytes = 0;
    g_target_debug.pkt_ok = 0;
    g_target_debug.pkt_checksum_error = 0;
    g_target_debug.pkt_range_error = 0;
    g_target_debug.timeout_count = 0;
    g_target_debug.dma_restart_count = 0;
    g_target_debug.uart_error_count = 0;
}

void protocol_feed(uint8_t byte)
{
    uint8_t i, sum = 0;
    int16_t dx, dy, roll;
    g_target_debug.rx_bytes++;
    if (s_count == 0) {
        if (byte == 0xAAu) { s_frame[0] = byte; s_count = 1; }
        return;
    }
    if (s_count == 1) {
        if (byte == 0x01u) { s_frame[1] = byte; s_count = 2; }
        else if (byte != 0xAAu) { s_count = 0; }
        return;
    }
    s_frame[s_count++] = byte;
    if (s_count < 9) return;
    s_count = 0;
    for (i = 0; i < 8; ++i) sum = (uint8_t)(sum + s_frame[i]);
    if (sum != s_frame[8]) {
        g_target_debug.pkt_checksum_error++;
        if (byte == 0xAAu) { s_frame[0] = byte; s_count = 1; }
        return;
    }
    dx = signed_be16(s_frame[2], s_frame[3]);
    dy = signed_be16(s_frame[4], s_frame[5]);
    roll = signed_be16(s_frame[6], s_frame[7]);
    if (dx < -1000 || dx > 1000 || dy < -1000 || dy > 1000 ||
        roll < -1000 || roll > 1000) {
        g_target_debug.pkt_range_error++;
        return;
    }
    taskENTER_CRITICAL();
    g_target.dx_raw = dx;
    g_target.dy_raw = dy;
    g_target.roll_raw = roll;
    g_target.dx = (float)dx * 0.001f;
    g_target.dy = (float)dy * 0.001f;
    g_target.roll = (float)roll * 0.001f;
    g_target.last_ms = HAL_GetTick();
    g_target.seq++;
    g_target.valid = 1;
    taskEXIT_CRITICAL();
    g_target_debug.pkt_ok++;
}

void protocol_update_timeout(void)
{
    if (g_target.valid &&
        (uint32_t)(HAL_GetTick() - g_target.last_ms) > TARGET_LOST_TIMEOUT_MS) {
        taskENTER_CRITICAL();
        g_target.valid = 0;
        g_target.dx_raw = g_target.dy_raw = g_target.roll_raw = 0;
        g_target.dx = g_target.dy = g_target.roll = 0.0f;
        taskEXIT_CRITICAL();
        g_target_debug.timeout_count++;
    }
}

void protocol_copy_target(target_state_t *out)
{
    if (out == 0) return;
    taskENTER_CRITICAL();
    out->dx_raw = g_target.dx_raw;
    out->dy_raw = g_target.dy_raw;
    out->roll_raw = g_target.roll_raw;
    out->dx = g_target.dx;
    out->dy = g_target.dy;
    out->roll = g_target.roll;
    out->last_ms = g_target.last_ms;
    out->seq = g_target.seq;
    out->valid = g_target.valid;
    taskEXIT_CRITICAL();
}
