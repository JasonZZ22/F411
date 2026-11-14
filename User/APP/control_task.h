#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

typedef struct {
    int bmi088_status;
    int imu_status;
    uint32_t imu_samples;
    uint32_t uart_rx_bytes;
    uint32_t uart_ok_frames;
    uint32_t uart_bad_checksums;
    uint32_t uart_out_of_range;
    uint32_t uart_errors;
    uint32_t uart_timeouts;
    uint8_t imu_ready;
    uint8_t imu_sampling;
    uint8_t uart_frame_valid;
    uint8_t roll_active;
    float roll_deg;
    float roll_rate_dps;
    float roll_error_deg;
    float roll_rate_target_dps;
    float roll_output_us;
    uint16_t servo_us[4];
} bench_diag_t;

extern volatile bench_diag_t g_bench_diag;
void Control_task(void);

#endif
