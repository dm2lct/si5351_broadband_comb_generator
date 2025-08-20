#ifndef UART_H
#define UART_H

#include <avr/io.h>

extern void uart_init (void);
extern int uart_putc (const uint8_t);
extern char uart_getc (void);

static inline void uart_flush (void)
{
	while (UCSR0B & (1 << UDRIE0));
}

#endif /* UART_H */
