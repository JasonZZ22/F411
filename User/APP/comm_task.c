#include "comm_task.h"
#include "protocol.h"
#include "ins_task.h"
#include "main.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;

#define USART6_RX_DMA_SIZE 256u
/* R5 = 300 kΩ、R6 = 100 kΩ，PA4 上的电压为 7V 电源网的 1/4。 */
#define BATTERY_DIVIDER_RATIO 4u
#define ADC_VREF_MV 3300u
#define ADC_FULL_SCALE 4095u
#define BATTERY_SAMPLE_PERIOD_MS 20u
#define BATTERY_ADC_TIMEOUT_MS 5u
#define BATTERY_ALARM_THRESHOLD_MV 3700u
/* 蜂鸣器总开关：1 允许低压报警鸣响，0 静音；电压检测仍正常运行。 */
#define BUZZER_ENABLE 0u

/* 上位机录制触发：每次加速度事件发送一个原始字节 0x52 ('R')。 */
#define RECORD_UART_SEND_ENABLE 1u /* 1：发送 0x52；0：禁止发送，仍检测加速度。 */
#define RECORD_UART_TRIGGER_BYTE 0x52u
#define RECORD_UART_RETRY_MS 20u

static volatile uint8_t s_rx_dma[USART6_RX_DMA_SIZE];
static volatile uint8_t s_restart_rx;
static uint16_t s_read_pos;
volatile battery_monitor_diag_t g_battery_monitor;
/* Keil Watch: 0 使用真实 ADC 电压，非 0 时用此毫伏值测试报警判断。 */
volatile uint32_t g_battery_test_voltage_mv;
volatile uint32_t g_record_uart_tx_count;
volatile uint32_t g_record_uart_tx_errors;

static void send_record_trigger(void)
{
#if RECORD_UART_SEND_ENABLE
    static uint32_t sent_event_count;
    static uint32_t last_attempt_ms;
    uint8_t command = RECORD_UART_TRIGGER_BYTE;
    const uint32_t now = HAL_GetTick();

    if (sent_event_count == g_record_accel_events) return;
    if (last_attempt_ms != 0u &&
        (uint32_t)(now - last_attempt_ms) < RECORD_UART_RETRY_MS) return;
    last_attempt_ms = now;
    if (HAL_UART_Transmit(&huart6, &command, 1u, 5u) == HAL_OK) {
        sent_event_count++;
        g_record_uart_tx_count++;
    } else {
        g_record_uart_tx_errors++;
    }
#endif
}

static void set_buzzer(uint8_t on)
{
    /* PB6/TIM4_CH1 经 Q2 驱动蜂鸣器；50% 占空比发出 4 kHz 音调。 */
    const uint32_t pulse = (BUZZER_ENABLE && on) ?
        (htim4.Init.Period + 1u) / 2u : 0u;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
}

static void monitor_battery(void)
{
    static uint32_t last_sample_ms;
    static uint32_t filtered_mv;
    static uint8_t filter_ready;
    uint32_t raw;
    uint32_t measured_mv;
    uint32_t alarm_mv;
    const uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_sample_ms) < BATTERY_SAMPLE_PERIOD_MS) return;
    last_sample_ms = now;

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        g_battery_monitor.adc_valid = 0u;
        g_battery_monitor.adc_errors++;
        return;
    }
    if (HAL_ADC_PollForConversion(&hadc1, BATTERY_ADC_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        g_battery_monitor.adc_valid = 0u;
        g_battery_monitor.adc_errors++;
        return;
    }
    raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* 12 位 ADC：V电源 = raw / 4095 * 3.3 V * 4。 */
    measured_mv = (raw * ADC_VREF_MV * BATTERY_DIVIDER_RATIO +
                   ADC_FULL_SCALE / 2u) / ADC_FULL_SCALE;
    if (!filter_ready) {
        filtered_mv = measured_mv;
        filter_ready = 1u;
    } else {
        filtered_mv = (filtered_mv * 3u + measured_mv + 2u) / 4u;
    }

    g_battery_monitor.adc_raw = (uint16_t)raw;
    g_battery_monitor.voltage_mv = filtered_mv;
    g_battery_monitor.sample_ms = now;
    g_battery_monitor.adc_valid = 1u;

    alarm_mv = g_battery_test_voltage_mv;
    if (alarm_mv == 0u) alarm_mv = filtered_mv;
    /* 整组电源严格低于 3.7 V 报警，高于 3.7 V 恢复；相等时保持原状态。 */
    if (!g_battery_monitor.low_voltage && alarm_mv < BATTERY_ALARM_THRESHOLD_MV) {
        g_battery_monitor.low_voltage = 1u;
        set_buzzer(1u);
    } else if (g_battery_monitor.low_voltage && alarm_mv > BATTERY_ALARM_THRESHOLD_MV) {
        g_battery_monitor.low_voltage = 0u;
        set_buzzer(0u);
    }
}

static HAL_StatusTypeDef start_rx(void)
{
    HAL_StatusTypeDef rc = HAL_UART_Receive_DMA(&huart6,
                                                (uint8_t *)s_rx_dma,
                                                USART6_RX_DMA_SIZE);
    if (rc == HAL_OK) {
        s_read_pos = 0;
        s_restart_rx = 0;
        /* Position polling needs neither half-transfer nor full-transfer events. */
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }
    return rc;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        g_target_debug.uart_error_count++;
        s_restart_rx = 1;
    }
}

void Comm_task(void)
{
    uint16_t write_pos;
    protocol_init();
    set_buzzer(0u);
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    for (;;) {
        monitor_battery();
        send_record_trigger();
        if (s_restart_rx || huart6.RxState == HAL_UART_STATE_READY) {
            HAL_UART_AbortReceive(&huart6);
            g_target_debug.dma_restart_count++;
            if (start_rx() != HAL_OK) {
                protocol_update_timeout();
                osDelay(100);
                continue;
            }
        }
        /* DMA writes the circular buffer; the task owns the parser. */
        write_pos = USART6_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        if (write_pos >= USART6_RX_DMA_SIZE) write_pos = 0;
        while (s_read_pos != write_pos) {
            protocol_feed(s_rx_dma[s_read_pos]);
            s_read_pos = (uint16_t)((s_read_pos + 1u) % USART6_RX_DMA_SIZE);
        }
        protocol_update_timeout();
        osDelay(1);
    }
}
