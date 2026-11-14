#ifndef COMM_TASK_H
#define COMM_TASK_H

#include <stdint.h>

/* Keil Watch: ADC 采样值、换算后的电源电压及低压报警状态。 */
typedef struct {
    uint16_t adc_raw;
    uint32_t voltage_mv;
    uint32_t sample_ms;
    uint32_t adc_errors;
    uint8_t adc_valid;
    uint8_t low_voltage;
} battery_monitor_diag_t;

extern volatile battery_monitor_diag_t g_battery_monitor;
/* Keil Watch：0x52 录像触发字节的发送成功和失败次数。 */
extern volatile uint32_t g_record_uart_tx_count;
extern volatile uint32_t g_record_uart_tx_errors;
void Comm_task(void);

#endif
