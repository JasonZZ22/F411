#include "control_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "protocol.h"

/* 控制任务每轮等待的 RTOS tick 数；当前 1 tick = 1 ms，约每 5 ms 运行一次。 */
#define CONTROL_PERIOD_TICKS 5u
/* 仅用于诊断：超过此时长未见新的 BMI088 采样，imu_sampling 置为 0。 */
#define IMU_INACTIVE_MS 100u
/* 控制用 roll 样本的最大允许年龄；超时后四路舵机回中位，单位 ms。 */
#define ROLL_SAMPLE_TIMEOUT_MS 30u

/* 正 roll 为从尾向镖头看顺时针；以下增益需根据飞行数据调试。 */
/* 需要保持的 roll 角，单位 °；当前目标为 0°。 */
#define ROLL_TARGET_DEG 0.0f
/* 角度环比例增益：roll 角误差（°）乘以该值，得到目标角速度（°/s）。 */
#define ROLL_ANGLE_KP_PER_S 2.0f
/* 角速度环比例增益：角速度误差（°/s）乘以该值，得到 PWM 修正量（μs）。 */
#define ROLL_RATE_KP_US_PER_DPS 0.4f
/* 角度环输出的目标角速度限幅，单位 °/s，正负方向共用。 */
#define ROLL_RATE_LIMIT_DPS 150.0f
/* PWM 修正量限幅，单位 μs；当前各路最终脉宽限制在 1400～1600 μs。 */
#define ROLL_PWM_LIMIT_US 100.0f
/* 四路舵机的中位脉宽，单位 μs；当前定时器每计数对应 1 μs。 */
#define SERVO_CENTER_US 1500.0f

/* 实测四路 PWM 同时增大产生负 roll 力矩，因此正 roll 指令要减小 PWM。 */
#define ROLL_MIX_1 (-1.0f) /* 1 号尾翼，TIM2_CH1 */
#define ROLL_MIX_2 (-1.0f) /* 2 号尾翼，TIM2_CH2 */
#define ROLL_MIX_3 (-1.0f) /* 3 号尾翼，TIM3_CH1 */
#define ROLL_MIX_4 (-1.0f) /* 4 号尾翼，TIM3_CH2 */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float wrap180f(float angle)
{
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static uint16_t servo_pulse(float correction_us, float mix)
{
    const float pulse = SERVO_CENTER_US + mix * correction_us;
    return (uint16_t)(clampf(pulse,
                            SERVO_CENTER_US - ROLL_PWM_LIMIT_US,
                            SERVO_CENTER_US + ROLL_PWM_LIMIT_US) + 0.5f);
}

static void write_servos(float roll_correction_us)
{
    const uint16_t servo1 = servo_pulse(roll_correction_us, ROLL_MIX_1);
    const uint16_t servo2 = servo_pulse(roll_correction_us, ROLL_MIX_2);
    const uint16_t servo3 = servo_pulse(roll_correction_us, ROLL_MIX_3);
    const uint16_t servo4 = servo_pulse(roll_correction_us, ROLL_MIX_4);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo4);

    g_bench_diag.servo_us[0] = servo1;
    g_bench_diag.servo_us[1] = servo2;
    g_bench_diag.servo_us[2] = servo3;
    g_bench_diag.servo_us[3] = servo4;
}

static void start_servos(void)
{
    write_servos(0.0f);
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

/* Read-only snapshot for Keil Watch. */
volatile bench_diag_t g_bench_diag;

void Control_task(void)
{
    uint32_t previous_count = 0u;
    uint32_t last_sample_ms = 0u;
    uint8_t saw_sample = 0u;

    start_servos();

    for (;;) {
        target_state_t uart_snapshot;
        roll_imu_sample_t roll_sample;
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
        ins_copy_roll_sample(&roll_sample);
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

        g_bench_diag.roll_active = 0u;
        g_bench_diag.roll_output_us = 0.0f;
        if (sensor_status == 0 && solver_status == 0 &&
            roll_sample.sequence != 0u &&
            (uint32_t)(now - roll_sample.sample_ms) <= ROLL_SAMPLE_TIMEOUT_MS) {
            const float error = wrap180f(ROLL_TARGET_DEG - roll_sample.roll_deg);
            const float rate_target = clampf(ROLL_ANGLE_KP_PER_S * error,
                                             -ROLL_RATE_LIMIT_DPS,
                                             ROLL_RATE_LIMIT_DPS);
            const float correction = clampf(
                ROLL_RATE_KP_US_PER_DPS * (rate_target - roll_sample.roll_rate_dps),
                -ROLL_PWM_LIMIT_US, ROLL_PWM_LIMIT_US);

            g_bench_diag.roll_active = 1u;
            g_bench_diag.roll_deg = roll_sample.roll_deg;
            g_bench_diag.roll_rate_dps = roll_sample.roll_rate_dps;
            g_bench_diag.roll_error_deg = error;
            g_bench_diag.roll_rate_target_dps = rate_target;
            g_bench_diag.roll_output_us = correction;
            write_servos(correction);
        } else {
            write_servos(0.0f);
        }

        osDelay(CONTROL_PERIOD_TICKS);
    }
}
