#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

typedef enum {
    CONTROL_STATE_ROLL_ONLY = 0,
    CONTROL_STATE_TARGET_TRACK = 1
} control_state_t;

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
    uint8_t control_state;
    uint8_t yaw_active;
    uint32_t target_age_ms;
    float roll_deg;
    float roll_rate_dps;
    float roll_error_deg;
    float roll_rate_target_dps;
    float roll_output_deg; /* roll 混控量，单位：尾翼相对中位角度。 */
    float roll_output_us;
    float target_dx;
    float yaw_rate_dps;
    float yaw_rate_target_dps;
    float yaw_output_deg; /* yaw 混控量，单位：尾翼相对中位角度。 */
    float yaw_output_us;
    float fin_deflection_deg[4]; /* 1500 μs 为 0°；正值表示该路 PWM 增大。 */
    float fin_angle_deg[4]; /* 按 1000～2500 μs -> 0～180° 标定的绝对角度。 */
    uint16_t servo_us[4];
} bench_diag_t;

extern volatile bench_diag_t g_bench_diag;
void Control_task(void);

#endif
