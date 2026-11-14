#include "comm_task.h"
#include "protocol.h"
#include "main.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart6;
#define USART6_RX_DMA_SIZE 256u
static volatile uint8_t s_rx_dma[USART6_RX_DMA_SIZE];
static volatile uint8_t s_restart_rx;
static uint16_t s_read_pos;

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
    for (;;) {
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
