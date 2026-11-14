#include "ins_task.h"
#include "imu_solver.h"
#include "cmsis_os.h"

/* Keil Watch: status 0 = ready; angles are in degrees. */
volatile int bmi088_status = 1;
volatile uint32_t bmi088_sample_count = 0;
volatile int imu_status = 1;
volatile float imu_roll_deg = 0.0f;
volatile float imu_pitch_deg = 0.0f;
volatile float imu_yaw_deg = 0.0f;
volatile bmi088_vec3_t bmi088_accel_ms2;
volatile bmi088_vec3_t bmi088_gyro_rads;

void INS_task(void)
{
    const bmi088_config_t cfg = BMI088_DEFAULT_CONFIG;
    bmi088_vec3_t accel, gyro;
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
            imu_status = imu_solver_init();
            if (imu_status != 0) {
                osDelay(1000);
                continue;
            }
        }
        imu_solver_update();
        {
            const imu_attitude_t *att = imu_solver_get();
            imu_roll_deg = att->roll;
            imu_pitch_deg = att->pitch;
            imu_yaw_deg = att->yaw;
        }
        if (bmi088_read_accel(&accel) == 0 && bmi088_read_gyro(&gyro) == 0) {
            bmi088_accel_ms2.x = accel.x;
            bmi088_accel_ms2.y = accel.y;
            bmi088_accel_ms2.z = accel.z;
            bmi088_gyro_rads.x = gyro.x;
            bmi088_gyro_rads.y = gyro.y;
            bmi088_gyro_rads.z = gyro.z;
            bmi088_sample_count++;
        } else {
            bmi088_status = -5;
            imu_status = 1;
        }
        osDelay(5);
    }
}
