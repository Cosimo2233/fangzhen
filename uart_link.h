#ifndef UART_LINK_H
#define UART_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void uart_link_init(void);
bool uart_link_read_line(char *line, size_t line_size);
void uart_link_write(const char *text);
uint32_t uart_link_overrun_count(void);

#endif
