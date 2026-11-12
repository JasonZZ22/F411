/**
  ******************************************************************************
  * @file    bmi088.h
  * @brief   Bosch BMI088 六轴 IMU 驱动（SPI）
  *
  *          BMI088 内部是两颗独立的芯片（两个 die），共用 SCK/MOSI/MISO，
  *          但各自有独立的片选：
  *            加速度计 (accel) -> CS_ACCEL -> PB2   数据就绪中断 -> INT_ACCEL (PC13)
  *            陀螺仪   (gyro)  -> CS_GYRO  -> PB3   数据就绪中断 -> INT_GYRO  (PC14)
  *          所以下面所有读写接口都要指定 die。
  *
  *          ⚠️ 一个著名的坑：加速度计用 SPI 读寄存器时，地址之后要先发一个
  *             dummy 字节才能读到真正的数据；陀螺仪不需要。
  *             本驱动内部已经处理好了，你不用管。
  *
  *          SPI1 配置：模式 3（CPOL=1, CPHA=1），6 MHz，8bit，MSB first。
  *          BMI088 上限 10MHz，所以 6.25MHz 是安全的。
  *
  *          典型用法：
  *            bmi088_config_t cfg = BMI088_DEFAULT_CONFIG;
  *            if (bmi088_init(&cfg) == 0) { ... }
  *
  *            bmi088_vec3_t acc, gyr;
  *            bmi088_read_accel(&acc);     // 单位 m/s^2
  *            bmi088_read_gyro(&gyr);      // 单位 rad/s
  ******************************************************************************
  */

#ifndef __BMI088_H__
#define __BMI088_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

/* ----------------------------------------------------------------------------
 * 公开的寄存器地址（想直接读寄存器时用得上）
 * --------------------------------------------------------------------------*/

#define BMI088_REG_ACC_CHIP_ID          0x00
#define BMI088_ACC_CHIP_ID_VALUE        0x1E

#define BMI088_REG_ACC_DATA             0x12    /**< X_LSB，连续 6 字节 */
#define BMI088_REG_ACC_CONF             0x40
#define BMI088_REG_ACC_RANGE            0x41
#define BMI088_REG_ACC_TEMP             0x22    /**< LSB，连续 2 字节 */
#define BMI088_REG_ACC_INT1_IO_CTRL     0x53
#define BMI088_REG_ACC_INT_MAP_DATA     0x58
#define BMI088_REG_ACC_PWR_CONF         0x7C
#define BMI088_REG_ACC_PWR_CTRL         0x7D
#define BMI088_REG_ACC_SOFTRESET        0x7E

#define BMI088_REG_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_CHIP_ID_VALUE       0x0F

#define BMI088_REG_GYRO_DATA            0x02    /**< X_LSB，连续 6 字节 */
#define BMI088_REG_GYRO_RANGE           0x0F
#define BMI088_REG_GYRO_BANDWIDTH       0x10
#define BMI088_REG_GYRO_LPM1            0x11
#define BMI088_REG_GYRO_SOFTRESET       0x14
#define BMI088_REG_GYRO_INT_CTRL        0x15
#define BMI088_REG_GYRO_INT3_INT4_CONF  0x16
#define BMI088_REG_GYRO_INT3_INT4_MAP   0x18

#define BMI088_SOFTRESET_CMD            0xB6

/* ----------------------------------------------------------------------------
 * 类型
 * --------------------------------------------------------------------------*/

/** 加速度计 / 陀螺仪，二选一 */
typedef enum
{
    BMI088_ACCEL = 0,
    BMI088_GYRO  = 1
} bmi088_die_t;

/** 三轴数据 */
typedef struct
{
    float x;
    float y;
    float z;
} bmi088_vec3_t;

/** 加速度计量程 */
typedef enum
{
    BMI088_ACCEL_RANGE_3G = 0x00,
    BMI088_ACCEL_RANGE_6G = 0x01,
    BMI088_ACCEL_RANGE_12G = 0x02,
    BMI088_ACCEL_RANGE_24G = 0x03
} bmi088_accel_range_t;

/** 陀螺仪量程 */
typedef enum
{
    BMI088_GYRO_RANGE_2000DPS = 0x00,
    BMI088_GYRO_RANGE_1000DPS = 0x01,
    BMI088_GYRO_RANGE_500DPS  = 0x02,
    BMI088_GYRO_RANGE_250DPS  = 0x03,
    BMI088_GYRO_RANGE_125DPS  = 0x04
} bmi088_gyro_range_t;

/** 初始化参数 */
typedef struct
{
    bmi088_accel_range_t accel_range;
    bmi088_gyro_range_t  gyro_range;
} bmi088_config_t;

/** 默认配置：加速度 ±6g，陀螺仪 ±2000dps（飞镖这种高动态场景建议就用这个） */
#define BMI088_DEFAULT_CONFIG   { BMI088_ACCEL_RANGE_6G, BMI088_GYRO_RANGE_2000DPS }

/* ----------------------------------------------------------------------------
 * 接口
 * --------------------------------------------------------------------------*/

/**
  * @brief  初始化 BMI088（两颗 die 都配）
  * @param  cfg  量程配置，可以直接用 BMI088_DEFAULT_CONFIG
  * @retval 0   成功
  *         -1  cfg 为 NULL
  *         -2  加速度计 CHIP_ID 读不到 / 不对（SPI 或 CS 有问题）
  *         -3  陀螺仪 CHIP_ID 读不到 / 不对
  *         -4  SPI 传输失败
  * @note   必须在 MX_SPI1_Init() 和 MX_GPIO_Init() 之后调用。
  */
int  bmi088_init(const bmi088_config_t *cfg);

/**
  * @brief  自检：只读两个 CHIP_ID，不改变任何配置
  * @retval 0 两颗都在
  */
int  bmi088_self_test(void);

/* -------- 原始值（ADC 码值，做标定/查手册时用） -------- */
int  bmi088_read_accel_raw(int16_t out[3]);
int  bmi088_read_gyro_raw(int16_t out[3]);

/* -------- 换算后的物理量 -------- */
/** @brief 读加速度，单位 m/s^2 */
int  bmi088_read_accel(bmi088_vec3_t *out);
/** @brief 读角速度，单位 rad/s */
int  bmi088_read_gyro(bmi088_vec3_t *out);
/** @brief 读温度，单位 ℃（来自加速度计那颗 die） */
int  bmi088_read_temperature(float *out);

/* -------- 通用寄存器读写（想调别的寄存器时用） -------- */
int  bmi088_read_reg(bmi088_die_t die, uint8_t reg, uint8_t *val);
int  bmi088_write_reg(bmi088_die_t die, uint8_t reg, uint8_t val);

/* -------- 数据就绪中断（INT_ACCEL / INT_GYRO） --------
 * 说明：不清中断标志，只记录“自上次调用以来发生过没有”。
 *       中断服务函数里置位，主循环里查询即可，不需要额外配 EXTI。
 */
int  bmi088_accel_drdy_take(void);      /**< 读并清除加速度计就绪标志 */
int  bmi088_gyro_drdy_take(void);       /**< 读并清除陀螺仪就绪标志 */

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H__ */
