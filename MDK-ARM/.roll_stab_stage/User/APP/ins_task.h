#ifndef INS_TASK_H
#define INS_TASK_H
#include <stdint.h>
#include "bmi088.h"
extern volatile int bmi088_status;
extern volatile uint32_t bmi088_sample_count;
extern volatile int imu_status;
extern volatile float imu_roll_deg, imu_pitch_deg, imu_yaw_deg;
extern volatile bmi088_vec3_t bmi088_accel_ms2, bmi088_gyro_rads;

typedef struct {
    float roll_deg;
    float roll_rate_dps;
    uint32_t sample_ms;
    uint32_t sequence;
} roll_imu_sample_t;

/* 在任务临界区内复制同一次解算的角度和角速度。 */
void ins_copy_roll_sample(roll_imu_sample_t *out);
void INS_task(void);
#endif
