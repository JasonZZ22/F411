/**
  ******************************************************************************
  * @file    imu_solver.c
  * @brief   姿态解算实现（互补滤波）
  ******************************************************************************
  */

#include "imu_solver.h"
#include "bmi088.h"
#include "cmsis_os.h"

#include <math.h>
#include <string.h>

/* ----------------------------------------------------------------------------
 * 内部状态
 * --------------------------------------------------------------------------*/

#define IMU_RAD2DEG     57.29577951f

/* 没加 static，是为了让 Keil 调试时能在任意作用域 Watch 到它。
 * 全工程没有重名，不加 static 不会有冲突。 */
imu_attitude_t s_att;

/** 陀螺零偏，单位 度/秒，已经是"轴符号修正之后"的坐标系 */
static float s_bias_dps[3];

static uint8_t s_inited;

/* 时间基准：优先用 DWT->CYCCNT（跑在 HCLK，10ns 一个 tick），
 * 万一 DWT 不可用就退回 HAL_GetTick()（只有 1ms 分辨率） */
static uint8_t  s_dwt_ok;
static uint32_t s_cyc_per_period;
static uint32_t s_last_cyc;
static uint32_t s_last_tick;

/* ----------------------------------------------------------------------------
 * 内部函数
 * --------------------------------------------------------------------------*/

static float imu_wrap180(float deg)
{
    while (deg >= 180.0f)  deg -= 360.0f;
    while (deg < -180.0f)  deg += 360.0f;
    return deg;
}

/**
  * @brief  读一次 IMU，做单位换算和轴符号修正
  * @param  acc_g     输出加速度，单位 g
  * @param  gyro_dps  输出角速度，单位 度/秒（**没有**去零偏）
  * @retval 0 成功
  *
  * ⚠️ 如果 BMI088 相对板子是 90° 这类"两轴对调"的装配，就在这里换轴。
  *    比如芯片转了 90°（板子的 X 对应芯片的 Y）时应该是：
  *        acc_g[0] = a.y / IMU_SOLVER_G * IMU_ACC_SIGN_Y;
  *        acc_g[1] = -a.x / IMU_SOLVER_G * IMU_ACC_SIGN_X;
  *    三个轴要保持右手系，别只换两个。
  */
static int imu_read_signed(float acc_g[3], float gyro_dps[3])
{
    bmi088_vec3_t a;
    bmi088_vec3_t g;

    if (bmi088_read_accel(&a) != 0) return -1;
    if (bmi088_read_gyro(&g) != 0)  return -1;

    acc_g[0] = a.x / IMU_SOLVER_G * IMU_ACC_SIGN_X;
    acc_g[1] = a.y / IMU_SOLVER_G * IMU_ACC_SIGN_Y;
    acc_g[2] = a.z / IMU_SOLVER_G * IMU_ACC_SIGN_Z;

    gyro_dps[0] = g.x * IMU_RAD2DEG * IMU_GYRO_SIGN_X;
    gyro_dps[1] = g.y * IMU_RAD2DEG * IMU_GYRO_SIGN_Y;
    gyro_dps[2] = g.z * IMU_RAD2DEG * IMU_GYRO_SIGN_Z;

    return 0;
}

/**
  * @brief  取"距上次调用过了多久"，顺便限速 
  * @retval 1 到时间了，*dt_out 是实测秒数；0 还没到解算周期
  * @note   用无符号减法算差值，所以计数器翻转也不会算错。
  */
static uint8_t imu_get_dt(float *dt_out)
{
    uint32_t now;

    if (s_dwt_ok)
    {
        now = DWT->CYCCNT;

        if ((uint32_t)(now - s_last_cyc) < s_cyc_per_period) return 0u;

        *dt_out    = (float)(uint32_t)(now - s_last_cyc) / (float)SystemCoreClock;
        s_last_cyc = now;
    }
    else
    {
        now = HAL_GetTick();

        if ((uint32_t)(now - s_last_tick) < IMU_SOLVER_PERIOD_MS) return 0u;

        *dt_out     = (float)(uint32_t)(now - s_last_tick) * 0.001f;
        s_last_tick = now;
    }

    return 1u;
}

/* ----------------------------------------------------------------------------
 * 对外接口
 * --------------------------------------------------------------------------*/

int imu_solver_recalibrate(uint16_t ms)
{
    float    acc[3];
    float    gyro[3];
    float    sx = 0.0f;
    float    sy = 0.0f;
    float    sz = 0.0f;
    uint32_t n  = 0u;
    uint32_t t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < (uint32_t)ms)
    {
        if (imu_read_signed(acc, gyro) == 0)
        {
            sx += gyro[0];
            sy += gyro[1];
            sz += gyro[2];
            n++;
        }
        osDelay(1u);
    }

    if (n < 10u) return -1;         /* 一个都没读上来，SPI 或 BMI088 有问题 */

    s_bias_dps[0] = sx / (float)n;
    s_bias_dps[1] = sy / (float)n;
    s_bias_dps[2] = sz / (float)n;

    return 0;
}

int imu_solver_init(void)
{
    float acc[3];
    float gyro[3];

    memset(&s_att, 0, sizeof(s_att));
    s_bias_dps[0] = 0.0f;
    s_bias_dps[1] = 0.0f;
    s_bias_dps[2] = 0.0f;
    s_inited      = 0u;

    /* ---------- 微秒级时间基准：DWT 周期计数器 ---------- */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_dwt_ok = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u) ? 1u : 0u;

    s_cyc_per_period = (SystemCoreClock / 1000u) * IMU_SOLVER_PERIOD_MS;
    s_last_cyc       = DWT->CYCCNT;
    s_last_tick      = HAL_GetTick();

    /* ---------- 零偏标定（这段时间板子必须静止） ---------- */
    if (imu_solver_recalibrate(IMU_SOLVER_CALIB_MS) != 0) return -1;

    /* ---------- 用重力方向给 roll / pitch 一个初值 ----------
     * 不给的话它们要从 0 慢慢被拉过去，上电头几秒读数不准 */
    if (imu_read_signed(acc, gyro) == 0)
    {
        s_att.roll  = atan2f(acc[1], acc[2]) * IMU_RAD2DEG;
        s_att.pitch = atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2])) * IMU_RAD2DEG;
    }
    s_att.yaw = 0.0f;

    s_inited = 1u;
    return 0;
}

void imu_solver_update(void)
{
    float dt;
    float acc[3];
    float gyro[3];
    float roll_acc;
    float pitch_acc;
    float alpha;

    if (!s_inited) return;
    if (imu_get_dt(&dt) == 0u) return;

    /* 主循环被阻塞太久时截断，避免角度一下子跳一大截 */
    if (dt > IMU_SOLVER_MAX_DT_S) dt = IMU_SOLVER_MAX_DT_S;

    if (imu_read_signed(acc, gyro) != 0) return;

    /* ---------- 去零偏 ---------- */
    gyro[0] -= s_bias_dps[0];
    gyro[1] -= s_bias_dps[1];
    gyro[2] -= s_bias_dps[2];

    /* ---------- 加速度计算出的绝对倾角（重力参考，长期不漂） ---------- */
    roll_acc  = atan2f(acc[1], acc[2]) * IMU_RAD2DEG;
    pitch_acc = atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2])) * IMU_RAD2DEG;

    /* ---------- 互补滤波 ----------
     * alpha 用 tau/(tau+dt) 的形式算，这样以后改 IMU_SOLVER_PERIOD_MS
     * 不用跟着重调 tau。 */
    alpha = IMU_SOLVER_TAU_S / (IMU_SOLVER_TAU_S + dt);

    s_att.roll  = alpha * (s_att.roll  + gyro[0] * dt) + (1.0f - alpha) * roll_acc;
    s_att.pitch = alpha * (s_att.pitch + gyro[1] * dt) + (1.0f - alpha) * pitch_acc;

    /* ---------- yaw：没有绝对参考，纯积分，必然漂 ---------- */
    s_att.yaw = imu_wrap180(s_att.yaw + gyro[2] * dt);
}

const imu_attitude_t *imu_solver_get(void)
{
    return &s_att;
}

void imu_solver_reset_attitude(void)
{
    s_att.roll  = 0.0f;
    s_att.pitch = 0.0f;
    s_att.yaw   = 0.0f;
}

void imu_solver_set_yaw(float deg)
{
    s_att.yaw = imu_wrap180(deg);
}
