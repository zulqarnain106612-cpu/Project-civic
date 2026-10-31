#ifndef UART_DMA_H
#define UART_DMA_H
#include <stdint.h>
#include <stddef.h>

/* Initialise DMA2 Stream7 Channel4 for USART1 TX.
 * Must be called after host_stream_init(). */
void uart_dma_init(void);

/* Non-blocking write: copies data to TX ring buffer, kicks DMA if idle.
 * Replaces the blocking host_stream_write() polling loop.
 * Returns bytes actually queued (may be less than len if ring is full). */
size_t uart_dma_write(const char *data, size_t len);

/* Convenience: uart_dma_write for null-terminated string */
void uart_dma_puts(const char *str);

/* DMA2 Stream7 IRQ handler — declared for startup vector table */
void DMA2_Stream7_IRQHandler(void);

#define UART_TX_RING_SIZE 4096u
#endif
