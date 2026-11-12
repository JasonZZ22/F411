/**
  ******************************************************************************
  * @file    bmi088.c
  * @brief   Bosch BMI088 六轴 IMU 驱动实现（SPI1，模式 3，6MHz）
  ******************************************************************************
  */

#include "bmi088.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

#include <string.h>

/* ----------------------------------------------------------------------------
 * 引脚映射（按本板原理图）
 * --------------------------------------------------------------------------*/

#define BMI088_ACCEL_CS_PORT    GPIOB      /* PB2 */
#define BMI088_ACCEL_CS_PIN     GPIO_PIN_2  /* PB2 */
#define BMI088_GYRO_CS_PORT     GPIOB       /* PB3 */
#define BMI088_GYRO_CS_PIN      GPIO_PIN_3  /* PB3 */

/* ----------------------------------------------------------------------------
 * 可调参数
 * --------------------------------------------------------------------------*/

/** SPI 单次传输超时（ms） */
#define BMI088_SPI_TIMEOUT_MS   10

/** 单次 burst 最多读多少字节（三轴 = 6，温度 = 2） */
#define BMI088_MAX_BURST        8

/**
  * 加速度计 ACC_CONF (0x40) 的位域：
  *   bit7    acc_bwp[2]   —— 固定写 1
  *   bit6:4  acc_bwp[1:0] —— 0b010 = normal 滤波器
  *   bit3:0  acc_odr      —— 0x5=12.5Hz 0x6=25Hz 0x7=50Hz 0x8=100Hz
  *                           0x9=200Hz 0xA=400Hz 0xB=800Hz 0xC=1600Hz
  * ⚠️ acc_odr 这张表请对着 BMI088 数据手册核一遍（本驱动唯一没有十足把握的地方）。
  *    想换成 800Hz 就把下面的 400HZ 改成 800HZ。
  */
#define BMI088_ACC_BWP_NORMAL   0xA0    /* bit7 + bit6:4=0b010 */
#define BMI088_ACC_ODR_100HZ    0x08
#define BMI088_ACC_ODR_400HZ    0x0A
#define BMI088_ACC_ODR_800HZ    0x0B

#define BMI088_ACC_CONF_VALUE   (BMI088_ACC_BWP_NORMAL | BMI088_ACC_ODR_400HZ)

/**
  * 陀螺仪 GYRO_BANDWIDTH (0x10)：
  *   0x00 = 2000Hz ODR / 532Hz 滤波
  *   0x01 = 2000Hz / 230Hz
  *   0x02 = 1000Hz / 116Hz
  *   0x03 =  400Hz /  47Hz
  *   0x04 =  200Hz /  23Hz
  *   0x05 =  100Hz /  12Hz
  */
#define BMI088_GYRO_BW_VALUE    0x02    /* 1000Hz / 116Hz */

/**
  * INT 引脚电平极性。板上 EXTI 配的是上升沿触发，所以传感器要设成高有效。
  * ⚠️ 如果中断一直不触发，把这两个值改成 0 试试（有可能极性反了）。
  *    注意：主流程用的是轮询读取，不依赖中断，所以这里错了也不影响读数。
  */
#define BMI088_ACCEL_INT_ACTIVE_HIGH    0x08    /* 0x53 的 bit3 = int1_lvl */
#define BMI088_GYRO_INT_ACTIVE_HIGH     0x01    /* 0x16 的 bit0 = int3_lvl */

/* CS 拉高之后留一点恢复时间再操作下一次（BMI088 要求 CSB 之间有最小间隔） */
#define BMI088_CS_GAP()                                     \
    do {                                                    \
        volatile uint32_t _i;                               \
        for (_i = 0u; _i < 16u; _i++) { __NOP(); }          \
    } while (0)

/* ----------------------------------------------------------------------------
 * 内部状态
 * --------------------------------------------------------------------------*/

static float    s_accel_lsb_per_g   = 5460.0f;      /* ±6g  默认 */
static float    s_gyro_lsb_per_dps  = 16.384f;      /* ±2000dps 默认 */

static volatile uint8_t s_accel_drdy = 0;
static volatile uint8_t s_gyro_drdy  = 0;

/* Keil Watch: actual CHIP_ID bytes seen during initialization. */
volatile uint8_t bmi088_accel_chip_id = 0;
volatile uint8_t bmi088_accel_first_id = 0;
volatile uint32_t bmi088_spi_error = 0;
volatile uint8_t bmi088_gyro_chip_id = 0;

/* ----------------------------------------------------------------------------
 * 底层 SPI 收发
 * --------------------------------------------------------------------------*/

static inline void bmi088_cs_low(bmi088_die_t die)
{
    if (die == BMI088_ACCEL)
        HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_RESET);
}

static inline void bmi088_cs_high(bmi088_die_t die)
{
    if (die == BMI088_ACCEL)
        HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);

    BMI088_CS_GAP();
}

/**
  * @brief  连续读寄存器
  * @note   加速度计要比陀螺仪多发一个 dummy 字节，这里统一处理掉了
  */
static int bmi088_read_regs(bmi088_die_t die, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t  tx[BMI088_MAX_BURST + 2];
    uint8_t  rx[BMI088_MAX_BURST + 2];
    uint16_t n = 0;

    if (buf == NULL || len == 0 || len > BMI088_MAX_BURST) return -1;

    memset(tx, 0x00, sizeof(tx));
    memset(rx, 0x00, sizeof(rx));

    tx[n++] = (uint8_t)(reg | 0x80);            /* bit7 = 1 表示读 */

    if (die == BMI088_ACCEL)
    {
        tx[n++] = 0x00;                         /* 加速度计的 dummy 字节 */
    }

    bmi088_cs_low(die);

    if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(n + len),
                                BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_spi_error = HAL_SPI_GetError(&hspi1);
        bmi088_cs_high(die);
        return -1;
    }

    bmi088_cs_high(die);

    memcpy(buf, &rx[n], len);
    return 0;
}

/**
  * @brief  写单个寄存器
  */
static int bmi088_write_regs(bmi088_die_t die, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t tx[4];

    if (data == NULL || len != 1) return -1;

    tx[0] = (uint8_t)(reg & 0x7F);              /* bit7 = 0 表示写 */
    tx[1] = data[0];

    bmi088_cs_low(die);

    if (HAL_SPI_Transmit(&hspi1, tx, 2, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_cs_high(die);
        return -1;
    }

    bmi088_cs_high(die);
    return 0;
}

/* ----------------------------------------------------------------------------
 * 小工具
 * --------------------------------------------------------------------------*/

/** 6 字节小端 -> 三轴 int16 */
static void bmi088_parse_xyz(const uint8_t *p, int16_t out[3])
{
    out[0] = (int16_t)(((uint16_t)p[1] << 8) | p[0]);
    out[1] = (int16_t)(((uint16_t)p[3] << 8) | p[2]);
    out[2] = (int16_t)(((uint16_t)p[5] << 8) | p[4]);
}

static float bmi088_accel_lsb_of(bmi088_accel_range_t r)
{
    switch (r)
    {
        case BMI088_ACCEL_RANGE_3G:  return 10920.0f;
        case BMI088_ACCEL_RANGE_6G:  return  5460.0f;
        case BMI088_ACCEL_RANGE_12G: return  2730.0f;
        case BMI088_ACCEL_RANGE_24G: return  1365.0f;
        default:                     return  5460.0f;
    }
}

static float bmi088_gyro_lsb_of(bmi088_gyro_range_t r)
{
    switch (r)
    {
        case BMI088_GYRO_RANGE_2000DPS: return  16.384f;
        case BMI088_GYRO_RANGE_1000DPS: return  32.768f;
        case BMI088_GYRO_RANGE_500DPS:  return  65.536f;
        case BMI088_GYRO_RANGE_250DPS:  return 131.072f;
        case BMI088_GYRO_RANGE_125DPS:  return 262.144f;
        default:                        return  16.384f;
    }
}

/* ----------------------------------------------------------------------------
 * 对外接口
 * --------------------------------------------------------------------------*/

int bmi088_read_reg(bmi088_die_t die, uint8_t reg, uint8_t *val)
{
    if (val == NULL) return -1;
    return bmi088_read_regs(die, reg, val, 1);
}

int bmi088_write_reg(bmi088_die_t die, uint8_t reg, uint8_t val)
{
    return bmi088_write_regs(die, reg, &val, 1);
}

int bmi088_self_test(void)
{
    uint8_t id;

    if (bmi088_read_reg(BMI088_ACCEL, BMI088_REG_ACC_CHIP_ID, &id) != 0) return -1;
    if (id != BMI088_ACC_CHIP_ID_VALUE) return -2;

    if (bmi088_read_reg(BMI088_GYRO, BMI088_REG_GYRO_CHIP_ID, &id) != 0) return -1;
    if (id != BMI088_GYRO_CHIP_ID_VALUE) return -3;

    return 0;
}

int bmi088_init(const bmi088_config_t *cfg)
{
    uint8_t id;

    if (cfg == NULL) return -1;

    /* CS 先确保是高（CubeMX 在 MX_GPIO_Init 里已经拉高了，这里再兜一次底） */
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT,  BMI088_GYRO_CS_PIN,  GPIO_PIN_SET);

    /* ================= 加速度计 ================= */

    /* After power-up the accel starts in I2C mode. One CS rising edge
       (dummy SPI read) switches it to SPI; check the ID on the next read. */
    if (bmi088_read_reg(BMI088_ACCEL, BMI088_REG_ACC_CHIP_ID, &id) != 0) return -4;
    bmi088_accel_first_id = id;
    if (bmi088_read_reg(BMI088_ACCEL, BMI088_REG_ACC_CHIP_ID, &id) != 0) return -4;
    bmi088_accel_chip_id = id;
    /* Probe the gyro even when the accel ID is wrong, for bus diagnosis. */
    if (bmi088_read_reg(BMI088_GYRO, BMI088_REG_GYRO_CHIP_ID, &id) == 0)
        bmi088_gyro_chip_id = id;
    if (bmi088_accel_chip_id != BMI088_ACC_CHIP_ID_VALUE) return -2;

    /* 软复位 */
    bmi088_write_reg(BMI088_ACCEL, BMI088_REG_ACC_SOFTRESET, BMI088_SOFTRESET_CMD);
    HAL_Delay(50);

    /* ⚠️⚠️ 软复位之后必须先做一次 dummy 读，把加速度计的 SPI 接口重新同步。
     *
     * BMI088 的加速度计上电默认处于 I2C 模式，要靠 CSB 的一个上升沿才会切到
     * SPI；数据手册明确写了：不做这一步的话 SPI **写** 会被忽略（读还能读到
     * CHIP_ID，所以看上去"通信正常"，很有迷惑性）。
     *
     * 症状就是：ACC_PWR_CTRL 读回 0x00（没打开）、ACC_PWR_CONF 读回 0x03
     * （还在 suspend）、数据寄存器全是 0 —— 而陀螺那边不受影响，因为陀螺的
     * 上电默认值刚好和本驱动的默认值一样，yaw 看起来是好的。 */
    bmi088_read_reg(BMI088_ACCEL, BMI088_REG_ACC_CHIP_ID, &id);

    /* 上电时序：先 PWR_CTRL 打开，再 PWR_CONF 退出 suspend，顺序不能反。
     *
     * ⚠️ BMI088 还要求两次写之间留出内部同步时间：normal 模式 ≥2us，
     *    suspend 模式 ≥1000us。原来的 BMI088_CS_GAP() 只有 16 个 NOP
     *    （约 0.16us），远远不够，所以下面每次写之间都插 HAL_Delay(1)
     *    （1ms，两种模式都满足）。开机多花几 ms，无所谓。 */
    bmi088_write_reg(BMI088_ACCEL, BMI088_REG_ACC_PWR_CTRL, 0x04);
    HAL_Delay(5);
    bmi088_write_reg(BMI088_ACCEL, BMI088_REG_ACC_PWR_CONF, 0x00);
    HAL_Delay(1);

    /* 带宽 / 量程 */
    bmi088_write_reg(BMI088_ACCEL, BMI088_REG_ACC_CONF,  (uint8_t)BMI088_ACC_CONF_VALUE);
    HAL_Delay(1);
    bmi088_write_reg(BMI088_ACCEL, BMI088_REG_ACC_RANGE, (uint8_t)cfg->accel_range);
    HAL_Delay(1);

    /* INT1：高有效（bit3 = int1_lvl），bit2 = int1_od 保持 0 即推挽。
     * 加速度计的 INT1 没有单独的“输出使能”位，把数据就绪映射到 INT1
     *（0x58 写 0x04）之后引脚就会自己驱动。 */
    bmi088_write_reg(BMI088_ACCEL, BMI088_REG_ACC_INT1_IO_CTRL,
                     (uint8_t)BMI088_ACCEL_INT_ACTIVE_HIGH);
    HAL_Delay(1);
    bmi088_write_reg(BMI088_ACCEL, BMI088_REG_ACC_INT_MAP_DATA, 0x04);

    HAL_Delay(10);

    s_accel_lsb_per_g = bmi088_accel_lsb_of(cfg->accel_range);

    /* ================= 陀螺仪 ================= */

    if (bmi088_read_reg(BMI088_GYRO, BMI088_REG_GYRO_CHIP_ID, &id) != 0) return -4;
    bmi088_gyro_chip_id = id;
    if (id != BMI088_GYRO_CHIP_ID_VALUE) return -3;

    /* 软复位 */
    bmi088_write_reg(BMI088_GYRO, BMI088_REG_GYRO_SOFTRESET, BMI088_SOFTRESET_CMD);
    HAL_Delay(30);

    /* 和加速度计一样，软复位后先 dummy 读一次重新同步接口 */
    bmi088_read_reg(BMI088_GYRO, BMI088_REG_GYRO_CHIP_ID, &id);

    /* 退出 suspend，进入 normal 模式 */
    bmi088_write_reg(BMI088_GYRO, BMI088_REG_GYRO_LPM1, 0x00);
    HAL_Delay(30);

    /* 两次写之间同样要留 ≥2us 的内部同步时间 */
    bmi088_write_reg(BMI088_GYRO, BMI088_REG_GYRO_RANGE,     (uint8_t)cfg->gyro_range);
    HAL_Delay(1);
    bmi088_write_reg(BMI088_GYRO, BMI088_REG_GYRO_BANDWIDTH, BMI088_GYRO_BW_VALUE);
    HAL_Delay(1);

    /* INT3：高有效 + 推挽；数据就绪映射到 INT3；使能数据就绪中断 */
    bmi088_write_reg(BMI088_GYRO, BMI088_REG_GYRO_INT3_INT4_CONF,
                     BMI088_GYRO_INT_ACTIVE_HIGH);
    HAL_Delay(1);
    bmi088_write_reg(BMI088_GYRO, BMI088_REG_GYRO_INT3_INT4_MAP, 0x01);
    HAL_Delay(1);
    bmi088_write_reg(BMI088_GYRO, BMI088_REG_GYRO_INT_CTRL,      0x80);

    HAL_Delay(10);

    s_gyro_lsb_per_dps = bmi088_gyro_lsb_of(cfg->gyro_range);

    /* 最后再确认一次两颗 die 都还在 */
    return bmi088_self_test();
}

int bmi088_read_accel_raw(int16_t out[3])
{
    uint8_t buf[6];

    if (out == NULL) return -1;
    if (bmi088_read_regs(BMI088_ACCEL, BMI088_REG_ACC_DATA, buf, 6) != 0) return -1;

    bmi088_parse_xyz(buf, out);
    return 0;
}

int bmi088_read_gyro_raw(int16_t out[3])
{
    uint8_t buf[6];

    if (out == NULL) return -1;
    if (bmi088_read_regs(BMI088_GYRO, BMI088_REG_GYRO_DATA, buf, 6) != 0) return -1;

    bmi088_parse_xyz(buf, out);
    return 0;
}

int bmi088_read_accel(bmi088_vec3_t *out)
{
    int16_t raw[3];
    const float g = 9.80665f;

    if (out == NULL) return -1;
    if (bmi088_read_accel_raw(raw) != 0) return -1;

    out->x = (float)raw[0] / s_accel_lsb_per_g * g;
    out->y = (float)raw[1] / s_accel_lsb_per_g * g;
    out->z = (float)raw[2] / s_accel_lsb_per_g * g;
    return 0;
}

int bmi088_read_gyro(bmi088_vec3_t *out)
{
    int16_t     raw[3];
    const float dps2rad = 0.0174532925f;     /* pi / 180 */

    if (out == NULL) return -1;
    if (bmi088_read_gyro_raw(raw) != 0) return -1;

    out->x = (float)raw[0] / s_gyro_lsb_per_dps * dps2rad;
    out->y = (float)raw[1] / s_gyro_lsb_per_dps * dps2rad;
    out->z = (float)raw[2] / s_gyro_lsb_per_dps * dps2rad;
    return 0;
}

int bmi088_read_temperature(float *out)
{
    uint8_t buf[2];
    int16_t raw;

    if (out == NULL) return -1;
    if (bmi088_read_regs(BMI088_ACCEL, BMI088_REG_ACC_TEMP, buf, 2) != 0) return -1;

    raw = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);

    /* 加速度计温度：0.125 ℃/LSB，零点偏置 23 ℃ */
    *out = (float)raw * 0.125f + 23.0f;
    return 0;
}

/* ----------------------------------------------------------------------------
 * 数据就绪标志
 * --------------------------------------------------------------------------*/

int bmi088_accel_drdy_take(void)
{
    uint8_t v = s_accel_drdy;
    s_accel_drdy = 0;
    return (int)v;
}

int bmi088_gyro_drdy_take(void)
{
    uint8_t v = s_gyro_drdy;
    s_gyro_drdy = 0;
    return (int)v;
}

/* ----------------------------------------------------------------------------
 * EXTI 回调
 *
 * HAL_GPIO_EXTI_Callback 是 HAL 里的 __weak 函数，这里给出强定义。
 * 如果你以后要处理别的 EXTI 引脚，请把下面这段搬到你自己的文件里，
 * 否则会出现重复定义。
 * --------------------------------------------------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        s_accel_drdy = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_14)
    {
        s_gyro_drdy = 1;
    }
}
