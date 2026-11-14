#include "control_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "protocol.h"

#define DIAG_PERIOD_TICKS 20u
#define IMU_INACTIVE_MS 100u

/* Read-only snapshot for Keil Watch. */
volatile bench_diag_t g_bench_diag;

void Control_task(void)
{
    uint32_t previous_count = 0u;
    uint32_t last_sample_ms = 0u;
    uint8_t saw_sample = 0u;

    for (;;) {
        target_state_t uart_snapshot;
        const uint32_t now = HAL_GetTick();
        const uint32_t count = bmi088_sample_count;
        const int sensor_status = bmi088_status;
        const int solver_status = imu_status;

        if (count != previous_count) {
            previous_count = count;
            last_sample_ms = now;
            saw_sample = 1u;
        }

        protocol_copy_target(&uart_snapshot);
        g_bench_diag.bmi088_status = sensor_status;
        g_bench_diag.imu_status = solver_status;
        g_bench_diag.imu_samples = count;
        g_bench_diag.imu_ready = (sensor_status == 0 && solver_status == 0);
        g_bench_diag.imu_sampling = saw_sample &&
            ((uint32_t)(now - last_sample_ms) < IMU_INACTIVE_MS);
        g_bench_diag.uart_frame_valid = uart_snapshot.valid;
        g_bench_diag.uart_rx_bytes = g_target_debug.rx_bytes;
        g_bench_diag.uart_ok_frames = g_target_debug.pkt_ok;
        g_bench_diag.uart_bad_checksums = g_target_debug.pkt_checksum_error;
        g_bench_diag.uart_out_of_range = g_target_debug.pkt_range_error;
        g_bench_diag.uart_errors = g_target_debug.uart_error_count;
        g_bench_diag.uart_timeouts = g_target_debug.timeout_count;

        osDelay(DIAG_PERIOD_TICKS);
    }
}
