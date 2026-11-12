/**
  ******************************************************************************
  * @file    imu_solver.h
  * @brief   BMI088 姿态解算：互补滤波出 roll / pitch，陀螺积分出 yaw
  *
  *          数据链：
  *            bmi088_read_accel / read_gyro
  *              -> 去零偏 + 轴符号修正
  *              -> 加速度计算绝对倾角（重力参考，长期不漂）
  *              -> 陀螺积分（短期准，长期漂）
  *              -> 互补滤波合成 roll / pitch
  *              -> yaw 只靠陀螺积分（BMI088 没有磁力计）
  *
  *          ⚠️ yaw 会持续漂移，这是物理限制，不是 bug。BMI088 里没有磁力计，
  *             yaw 没有绝对参考。标定好的话典型漂移是几度/分钟。要绝对 yaw
  *             必须外加磁力计，或者用机械限位 / 视觉给个基准，
  *             然后调 imu_solver_set_yaw() 校一次。
  *
  *          数据存在哪：解算结果在内部静态结构体里，imu_solver_get() 返回它的地址。
  *                     任务中随时取，不用自己缓存。
  *
  *          典型用法：
  *            if (imu_solver_init() == 0) { ... }   // 标定这段时间板子必须静止
  *            while (1) {
  *                imu_solver_update();              // 内部自我限速，随便调
  *                const imu_attitude_t *att = imu_solver_get();
  *                // att->roll / att->pitch / att->yaw，单位都是度
  *            }
  ******************************************************************************
  */

#ifndef __IMU_SOLVER_H__
#define __IMU_SOLVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

/* ----------------------------------------------------------------------------
 * 可调参数
 * --------------------------------------------------------------------------*/

/** 解算周期（ms）。5 = 200Hz，与当前 sensorTask 的 5ms 周期对齐。
 *  imu_solver_update() 可以随便调，内部会按这个周期自我限速 */
#define IMU_SOLVER_PERIOD_MS        5u

/** 互补滤波时间常数（秒）。
 *  越大越信陀螺（跟得快，但加速度计的修正力弱、漂得久）
 *  越小越信加速度计（不漂，但振动时会跟着抖）
 *  0.5 ~ 2.0 是常用范围 */
#define IMU_SOLVER_TAU_S            0.5f

/** 上电零偏标定持续时长（ms）。这一段时间板子必须完全静止 */
#define IMU_SOLVER_CALIB_MS         500u

/** 单次 dt 上限（秒）。超过就按这个值截断，避免陀螺积分因为一次
 *  长阻塞（比如串口打印）跳一大截角度 */
#define IMU_SOLVER_MAX_DT_S         0.05f

/** 重力加速度常数 */
#define IMU_SOLVER_G                9.80665f

/* ----------------------------------------------------------------------------
 * 装配方向修正
 *
 * BMI088 芯片在板子上的朝向决定符号。先按数据手册默认轴向烧进去试：
 *   静止水平放置时 roll / pitch 应该都接近 0；
 *   绕着某个轴正向转，对应那个角的读数应该跟着往正方向走。
 * 哪个轴反了就把对应符号改成 -1.0f。
 *
 * ⚠️ 如果芯片相对板子是 90° 这类"两轴对调"的装配，光翻符号不够，
 *    得在 imu_solver.c 的 imu_read_signed() 里换轴，那个函数里有注释标了位置。
 * --------------------------------------------------------------------------*/

#define IMU_ACC_SIGN_X      (+1.0f)
#define IMU_ACC_SIGN_Y      (+1.0f)
#define IMU_ACC_SIGN_Z      (+1.0f)

#define IMU_GYRO_SIGN_X     (+1.0f)
#define IMU_GYRO_SIGN_Y     (+1.0f)
#define IMU_GYRO_SIGN_Z     (+1.0f)

/* ----------------------------------------------------------------------------
 * 输出
 * --------------------------------------------------------------------------*/

typedef struct
{
    float roll;     /**< 横滚角，度，绕 X 轴，互补滤波 */
    float pitch;    /**< 俯仰角，度，绕 Y 轴，互补滤波 */
    float yaw;      /**< 偏航角，度，绕 Z 轴，**纯积分会漂** */
} imu_attitude_t;

/* ----------------------------------------------------------------------------
 * 接口
 * --------------------------------------------------------------------------*/

/**
  * @brief  初始化解算器并做上电零偏标定
  * @retval 0   成功
  *         -1  标定期间采样太少（SPI 没通 / BMI088 没初始化）
  * @note   调用前必须先 bmi088_init() 成功。
  *         **调用时板子必须静止不动**，否则零偏会标歪，yaw 会漂得很快。
  */
int imu_solver_init(void);

/**
  * @brief  跑一次解算。由 sensorTask 周期调用即可，内部按 IMU_SOLVER_PERIOD_MS 限速。
  * @note   内部用 DWT 周期计数器做微秒级时间基准，dt 是实测值，
  *         所以即使主循环被串口打印阻塞，角度的积分总量依然是对的
  *         （只是会漏掉几个采样点）。
  */
void imu_solver_update(void);

/**
  * @brief  取当前姿态
  * @return 指向内部静态结构体的指针，不会返回 NULL。
  *         结构体每个周期被原地覆盖，不要长期持有后当历史数据用。
  */
const imu_attitude_t *imu_solver_get(void);

/**
  * @brief  重新做一次零偏标定（比如起飞前静止时再校一次）
  * @retval 0 成功，-1 采样失败
  */
int imu_solver_recalibrate(uint16_t ms);

/**
  * @brief  把 roll / pitch / yaw 全部清零
  * @note   roll / pitch 会被加速度计慢慢拉回真实值；yaw 清零后就是新的零点
  */
void imu_solver_reset_attitude(void);

/**
  * @brief  手动设定 yaw（用视觉或机械基准校一次漂移）
  */
void imu_solver_set_yaw(float deg);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_SOLVER_H__ */
