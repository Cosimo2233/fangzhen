#include "uart_link.h"

#include <string.h>

#include "artemis_config.h"
#include "ti_msp_dl_config.h"

#if (ARTEMIS_UART_RX_BUFFER_SIZE & (ARTEMIS_UART_RX_BUFFER_SIZE - 1U)) != 0U
#error "ARTEMIS_UART_RX_BUFFER_SIZE must be a power of two"
#endif

static volatile uint8_t rx_buffer[ARTEMIS_UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overruns;

/* 行缓冲只在主循环里使用；中断只负责把原始字节放入 rx_buffer。 */
static char line_buffer[ARTEMIS_UART_LINE_BUFFER_SIZE];
static size_t line_length;
static bool line_discarding;

static uint16_t next_index(uint16_t index)
{
    return (uint16_t) ((index + 1U) & (ARTEMIS_UART_RX_BUFFER_SIZE - 1U));
}

static bool pop_byte(uint8_t *value)
{
    const uint16_t tail = rx_tail;
    if (tail == rx_head) {
        return false;
    }
    *value = rx_buffer[tail];
    rx_tail = next_index(tail);
    return true;
}

void uart_link_init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    rx_overruns = 0U;
    line_length = 0U;
    line_discarding = false;
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

bool uart_link_read_line(char *line, size_t line_size)
{
    uint8_t value;

    if ((line == NULL) || (line_size == 0U)) {
        return false;
    }
    while (pop_byte(&value)) {
        if (value == '\r') {
            continue;
        }
        if (value == '\n') {
            /* 超长行会一直丢弃到下一个 LF，避免半帧进入协议解析。 */
            if (line_discarding) {
                line_discarding = false;
                line_length = 0U;
                continue;
            }
            if (line_length == 0U) {
                continue;
            }
            if (line_length >= line_size) {
                line_length = 0U;
                continue;
            }
            memcpy(line, line_buffer, line_length);
            line[line_length] = '\0';
            line_length = 0U;
            return true;
        }
        if (line_discarding) {
            continue;
        }
        if (line_length + 1U >= sizeof(line_buffer)) {
            line_discarding = true;
            line_length = 0U;
            continue;
        }
        line_buffer[line_length++] = (char) value;
    }
    return false;
}

void uart_link_write(const char *text)
{
    if (text == NULL) {
        return;
    }
    /* TX 使用阻塞发送。当前协议一帧很短，简单可靠，且不会占用动态内存。 */
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) *text);
        text++;
    }
}

uint32_t uart_link_overrun_count(void)
{
    return rx_overruns;
}

void UART_0_INST_IRQHandler(void)
{
    /* RX 中断尽量短，只搬运一个字节，解析工作留给主循环。 */
    if (DL_UART_Main_getPendingInterrupt(UART_0_INST) == DL_UART_MAIN_IIDX_RX) {
        const uint8_t value = DL_UART_Main_receiveData(UART_0_INST);
        const uint16_t head = rx_head;
        const uint16_t next = next_index(head);
        if (next == rx_tail) {
            rx_overruns++;
        } else {
            rx_buffer[head] = value;
            rx_head = next;
        }
    }
}
