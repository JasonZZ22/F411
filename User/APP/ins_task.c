#include "ins_task.h"
#include "imu_solver.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* 合加速度门限（m/s²，包含静止时约 9.81 m/s² 的重力）；按实测调整。 */
#define RECORD_ACCEL_THRESHOLD_MS2 30.0f
/* 回落到此值以下并持续 200 ms 后，允许下一次录像触发。 */
#define RECORD_ACCEL_REARM_THRESHOLD_MS2 12.0f
#define RECORD_ACCEL_REARM_MS 200u

/* Keil Watch: status 0 = ready; angles are in degrees. */
volatile int bmi088_status = 1;
volatile uint32_t bmi088_sample_count = 0;
volatile int imu_status = 1;
volatile float imu_roll_deg = 0.0f;
volatile float imu_pitch_deg = 0.0f;
volatile float imu_yaw_deg = 0.0f;
volatile bmi088_vec3_t bmi088_accel_ms2;
volatile bmi088_vec3_t bmi088_gyro_rads;
volatile float g_record_accel_ms2;
volatile uint8_t g_record_accel_triggered;
volatile uint32_t g_record_accel_events;
static roll_imu_sample_t s_roll_sample;

void ins_copy_roll_sample(roll_imu_sample_t *out)
{
    if (out == 0) return;
    taskENTER_CRITICAL();
    *out = s_roll_sample;
    taskEXIT_CRITICAL();
}

void INS_task(void)
{
    const bmi088_config_t cfg = BMI088_DEFAULT_CONFIG;
    bmi088_vec3_t accel, gyro;
    int update_status;
    uint8_t record_armed = 1u;
    uint8_t below_rearm = 0u;
    uint32_t below_since_ms = 0u;
    for (;;) {
        if (bmi088_status != 0) {
            bmi088_status = bmi088_init(&cfg);
            if (bmi088_status != 0) {
                imu_status = 1;
                osDelay(1000);
                continue;
            }
        }
        if (imu_status != 0) {
            /* Keep the board still during this 500 ms gyro calibration. */
            taskENTER_CRITICAL();
            s_roll_sample.sequence = 0u;
            s_roll_sample.sample_ms = 0u;
            taskEXIT_CRITICAL();
            imu_status = imu_solver_init();
            if (imu_status != 0) {
                osDelay(1000);
                continue;
            }
        }
        if (bmi088_read_accel(&accel) == 0 && bmi088_read_gyro(&gyro) == 0) {
            bmi088_accel_ms2.x = accel.x;
            bmi088_accel_ms2.y = accel.y;
            bmi088_accel_ms2.z = accel.z;
            bmi088_gyro_rads.x = gyro.x;
            bmi088_gyro_rads.y = gyro.y;
            bmi088_gyro_rads.z = gyro.z;
            bmi088_sample_count++;
            {
                const float accel_sq = accel.x * accel.x +
                                       accel.y * accel.y + accel.z * accel.z;
                const uint32_t now = HAL_GetTick();
                g_record_accel_ms2 = sqrtf(accel_sq);
                if (record_armed &&
                    accel_sq > RECORD_ACCEL_THRESHOLD_MS2 * RECORD_ACCEL_THRESHOLD_MS2) {
                    record_armed = 0u;
                    below_rearm = 0u;
                    g_record_accel_triggered = 1u;
                    g_record_accel_events++;
                } else if (!record_armed &&
                           accel_sq < RECORD_ACCEL_REARM_THRESHOLD_MS2 *
                                      RECORD_ACCEL_REARM_THRESHOLD_MS2) {
                    if (!below_rearm) {
                        below_rearm = 1u;
                        below_since_ms = now;
                    } else if ((uint32_t)(now - below_since_ms) >= RECORD_ACCEL_REARM_MS) {
                        record_armed = 1u;
                        below_rearm = 0u;
                        g_record_accel_triggered = 0u;
                    }
                } else {
                    below_rearm = 0u;
                }
            }
        } else {
            bmi088_status = -5;
            imu_status = 1;
            osDelay(5);
            continue;
        }
        update_status = imu_solver_update();
        if (update_status < 0) {
            bmi088_status = -5;
            imu_status = 1;
            osDelay(5);
            continue;
        }
        if (update_status > 0) {
            const imu_attitude_t *att = imu_solver_get();
            imu_roll_deg = att->roll;
            imu_pitch_deg = att->pitch;
            imu_yaw_deg = att->yaw;
            taskENTER_CRITICAL();
            s_roll_sample.roll_deg = att->roll;
            s_roll_sample.roll_rate_dps = att->roll_rate_dps;
            s_roll_sample.yaw_rate_dps = att->yaw_rate_dps;
            s_roll_sample.sample_ms = HAL_GetTick();
            s_roll_sample.sequence++;
            taskEXIT_CRITICAL();
        }
        osDelay(5);
    }
}
