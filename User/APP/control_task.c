#include "control_task.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "protocol.h"
#include <math.h>

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
/* 角速度误差（°/s）到尾翼偏转角（°）；原 0.4 μs/(°/s) 对应 0.048 °/(°/s)。 */
#define ROLL_RATE_KP_DEG_PER_DPS 0.096f
/* 角度环输出的目标角速度限幅，单位 °/s，正负方向共用。 */
#define ROLL_RATE_LIMIT_DPS 150.0f
/* roll 单独可使用的最大尾翼偏转角。 */
#define ROLL_DEFLECTION_LIMIT_DEG 15.0f
/* roll 与 yaw 混合后每片尾翼相对中位的最大偏转角。 */
#define FIN_DEFLECTION_LIMIT_DEG 15.0f
/* 实测尾翼角度标定：500～2500 μs 对应 0～180°，1500 μs 为装配中位。 */
#define SERVO_PULSE_MIN_US 500.0f
#define SERVO_PULSE_MAX_US 2500.0f
#define FIN_ANGLE_RANGE_DEG 180.0f
#define SERVO_CENTER_US 1500.0f
#define FIN_DEG_PER_US (FIN_ANGLE_RANGE_DEG / (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US))
#define SERVO_US_PER_DEG ((SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / FIN_ANGLE_RANGE_DEG)

/* 实测四路 PWM 同时增大产生负 roll 力矩，因此正 roll 指令要减小 PWM。 */
#define ROLL_MIX_1 (-1.0f) /* 1 号尾翼，TIM2_CH1 */
#define ROLL_MIX_2 (-1.0f) /* 2 号尾翼，TIM2_CH2 */
#define ROLL_MIX_3 (-1.0f) /* 3 号尾翼，TIM3_CH1 */
#define ROLL_MIX_4 (-1.0f) /* 4 号尾翼，TIM3_CH2 */

/* 目标帧的 dx 正值表示目标在画面左侧；yaw 以镖头向左为正。 */
#define TARGET_YAW_ENABLE 1u
#define YAW_TARGET_DX_SIGN (+1.0f)
/* 陀螺仪 Z 轴角速度应在镖头向左转动时为正，台架上需核对。 */
#define YAW_RATE_SIGN (+1.0f)
/* 实测：1、2 号 PWM 增大，3、4 号 PWM 减小，产生正 yaw 力矩。 */
#define YAW_MIX_SIGN (+1.0f)
#define YAW_MIX_1 (+YAW_MIX_SIGN)
#define YAW_MIX_2 (+YAW_MIX_SIGN)
#define YAW_MIX_3 (-YAW_MIX_SIGN)
#define YAW_MIX_4 (-YAW_MIX_SIGN)

/* 图像偏差 [-1,1] -> 目标 yaw 角速度（°/s），再由角速度误差产生尾翼偏转角。 */
#define YAW_IMAGE_KP_DPS 60.0f
#define YAW_RATE_LIMIT_DPS 60.0f
#define YAW_RATE_KP_DEG_PER_DPS 0.72f
#define YAW_DEFLECTION_LIMIT_DEG 36.0f
/* roll 偏差过大时，画面左右不再近似对应纯 yaw，暂缓 yaw 输出。 */
#define YAW_ROLL_GUARD_DEG 45.0f

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

static uint16_t servo_pulse(float roll_deg, float roll_mix,
                            float yaw_deg, float yaw_mix)
{
    const float deflection_deg = clampf(roll_mix * roll_deg + yaw_mix * yaw_deg,
                                         -FIN_DEFLECTION_LIMIT_DEG,
                                         FIN_DEFLECTION_LIMIT_DEG);
    const float pulse_us = SERVO_CENTER_US + deflection_deg * SERVO_US_PER_DEG;
    return (uint16_t)(clampf(pulse_us, SERVO_PULSE_MIN_US,
                            SERVO_PULSE_MAX_US) + 0.5f);
}

static void write_servos(float roll_correction_deg, float yaw_correction_deg)
{
    const uint16_t servo1 = servo_pulse(roll_correction_deg, ROLL_MIX_1,
                                        yaw_correction_deg, YAW_MIX_1);
    const uint16_t servo2 = servo_pulse(roll_correction_deg, ROLL_MIX_2,
                                        yaw_correction_deg, YAW_MIX_2);
    const uint16_t servo3 = servo_pulse(roll_correction_deg, ROLL_MIX_3,
                                        yaw_correction_deg, YAW_MIX_3);
    const uint16_t servo4 = servo_pulse(roll_correction_deg, ROLL_MIX_4,
                                        yaw_correction_deg, YAW_MIX_4);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo4);

    g_bench_diag.servo_us[0] = servo1;
    g_bench_diag.servo_us[1] = servo2;
    g_bench_diag.servo_us[2] = servo3;
    g_bench_diag.servo_us[3] = servo4;
    for (uint32_t i = 0u; i < 4u; ++i) {
        const float pulse_us = (float)g_bench_diag.servo_us[i];
        g_bench_diag.fin_deflection_deg[i] =
            (pulse_us - SERVO_CENTER_US) * FIN_DEG_PER_US;
        g_bench_diag.fin_angle_deg[i] =
            (pulse_us - SERVO_PULSE_MIN_US) * FIN_DEG_PER_US;
    }
}

static void start_servos(void)
{
    write_servos(0.0f, 0.0f);
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
    control_state_t state = CONTROL_STATE_ROLL_ONLY;

    start_servos();

    for (;;) {
        target_state_t uart_snapshot;
        roll_imu_sample_t roll_sample;
        float roll_correction_deg = 0.0f;
        float yaw_correction_deg = 0.0f;
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
        const uint32_t target_age_ms = uart_snapshot.seq ?
            (uint32_t)(now - uart_snapshot.last_ms) : 0u;
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
        g_bench_diag.target_age_ms = target_age_ms;
        g_bench_diag.target_dx = uart_snapshot.dx;

        g_bench_diag.roll_active = 0u;
        g_bench_diag.roll_output_deg = 0.0f;
        g_bench_diag.roll_output_us = 0.0f;
        g_bench_diag.yaw_active = 0u;
        g_bench_diag.yaw_rate_dps = 0.0f;
        g_bench_diag.yaw_rate_target_dps = 0.0f;
        g_bench_diag.yaw_output_deg = 0.0f;
        g_bench_diag.yaw_output_us = 0.0f;
        if (sensor_status == 0 && solver_status == 0 &&
            roll_sample.sequence != 0u &&
            (uint32_t)(now - roll_sample.sample_ms) <= ROLL_SAMPLE_TIMEOUT_MS) {
            const uint8_t target_fresh = uart_snapshot.valid &&
                uart_snapshot.seq != 0u &&
                target_age_ms <= TARGET_LOST_TIMEOUT_MS;
            const float error = wrap180f(ROLL_TARGET_DEG - roll_sample.roll_deg);
            const float rate_target = clampf(ROLL_ANGLE_KP_PER_S * error,
                                             -ROLL_RATE_LIMIT_DPS,
                                             ROLL_RATE_LIMIT_DPS);
            const float correction = clampf(
                ROLL_RATE_KP_DEG_PER_DPS * (rate_target - roll_sample.roll_rate_dps),
                -ROLL_DEFLECTION_LIMIT_DEG, ROLL_DEFLECTION_LIMIT_DEG);
            const float yaw_rate_dps = YAW_RATE_SIGN * roll_sample.yaw_rate_dps;

            state = target_fresh ? CONTROL_STATE_TARGET_TRACK :
                                   CONTROL_STATE_ROLL_ONLY;
            roll_correction_deg = correction;
            g_bench_diag.roll_active = 1u;
            g_bench_diag.roll_deg = roll_sample.roll_deg;
            g_bench_diag.roll_rate_dps = roll_sample.roll_rate_dps;
            g_bench_diag.roll_error_deg = error;
            g_bench_diag.roll_rate_target_dps = rate_target;
            g_bench_diag.roll_output_deg = correction;
            g_bench_diag.roll_output_us = correction * SERVO_US_PER_DEG;
            g_bench_diag.yaw_rate_dps = yaw_rate_dps;

            if (TARGET_YAW_ENABLE && state == CONTROL_STATE_TARGET_TRACK &&
                fabsf(error) <= YAW_ROLL_GUARD_DEG) {
                const float target_dx = clampf(
                    YAW_TARGET_DX_SIGN * uart_snapshot.dx, -1.0f, 1.0f);
                const float yaw_rate_target = clampf(
                    YAW_IMAGE_KP_DPS * target_dx,
                    -YAW_RATE_LIMIT_DPS, YAW_RATE_LIMIT_DPS);
                const float yaw_request = YAW_RATE_KP_DEG_PER_DPS *
                    (yaw_rate_target - yaw_rate_dps);
                const float roll_headroom = FIN_DEFLECTION_LIMIT_DEG -
                    fabsf(roll_correction_deg);
                const float yaw_limit = clampf(roll_headroom, 0.0f,
                                                YAW_DEFLECTION_LIMIT_DEG);

                yaw_correction_deg = clampf(yaw_request, -yaw_limit, yaw_limit);
                g_bench_diag.yaw_active = 1u;
                g_bench_diag.yaw_rate_target_dps = yaw_rate_target;
                g_bench_diag.yaw_output_deg = yaw_correction_deg;
                g_bench_diag.yaw_output_us = yaw_correction_deg * SERVO_US_PER_DEG;
            }
            write_servos(roll_correction_deg, yaw_correction_deg);
        } else {
            state = CONTROL_STATE_ROLL_ONLY;
            write_servos(0.0f, 0.0f);
        }
        g_bench_diag.control_state = (uint8_t)state;

        osDelay(CONTROL_PERIOD_TICKS);
    }
}
