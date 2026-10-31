/*
 * uart_dma.c — DMA2 Stream7 Channel4 TX ring buffer for USART1.
 *
 * Lock-free single-producer (main loop) / single-consumer (ISR) ring buffer.
 * DMA2 Stream7 Channel4 → USART1 DR, interrupt on transfer complete.
 *
 * Ring indices are uint16_t (0..UART_TX_RING_SIZE-1).
 * tx_head: written by producer (uart_dma_write), read by ISR/dma_kick.
 * tx_tail: advanced by ISR in DMA2_Stream7_IRQHandler, read by dma_kick.
 * dma_active: 1 while DMA is transferring, 0 when idle.
 *
 * dma_kick() always sends a contiguous segment (tail → head or ring end,
 * whichever comes first); the IRQ then calls dma_kick() again to handle
 * wrap-around or remaining bytes.
 */

#include "uart_dma.h"
#include "stm32f407xx.h"
#include <string.h>

/* ── ring buffer state ──────────────────────────────────────────────── */
static uint8_t           tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t tx_head;      /* producer index */
static volatile uint16_t tx_tail;      /* consumer index (DMA source)  */
static volatile uint8_t  dma_active;   /* 1 = DMA transfer in progress */

/* bytes committed to the current DMA transfer (needed in IRQ to advance tail) */
static volatile uint16_t dma_transfer_len;

/* ── internal: start next contiguous DMA transfer ───────────────────── */
static void dma_kick(void)
{
    /* Snapshot head/tail atomically (both fit in a register on Cortex-M4) */
    const uint16_t tail = tx_tail;
    const uint16_t head = tx_head;

    if (tail == head) return; /* nothing to send */

    /* Contiguous bytes available without crossing ring wrap */
    uint16_t contig;
    if (head > tail) {
        contig = (uint16_t)(head - tail);
    } else {
        contig = (uint16_t)(UART_TX_RING_SIZE - tail); /* send to ring end */
    }

    if (contig == 0u) return;

    dma_transfer_len = contig;
    dma_active       = 1u;

    /* Disable stream before reconfiguring */
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    /* Wait for stream to actually stop (RM0090 §10.5.6) */
    while (DMA2_Stream7->CR & DMA_SxCR_EN) { }

    /* Clear all interrupt flags for Stream7 (HIFCR bits 27,25,24,22,21) */
    DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 |
                  DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;

    DMA2_Stream7->M0AR = (uint32_t)(uintptr_t)(&tx_ring[tail]);
    DMA2_Stream7->NDTR = contig;
    DMA2_Stream7->CR  |= DMA_SxCR_EN;
}

/* ── uart_dma_init ───────────────────────────────────────────────────── */
void uart_dma_init(void)
{
    tx_head          = 0u;
    tx_tail          = 0u;
    dma_active       = 0u;
    dma_transfer_len = 0u;

    /* Enable DMA2 clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR; /* bus sync */

    /* Disable stream while configuring */
    DMA2_Stream7->CR = 0u;
    while (DMA2_Stream7->CR & DMA_SxCR_EN) { }

    /* Configure DMA2 Stream7:
     *   Channel 4      : bits [27:25] = 0b100
     *   DIR            : memory→peripheral (SxCR_DIR bit 6 = 1, bit 7 = 0)
     *   MINC           : memory increment enable
     *   No CIRC        : not circular
     *   PSIZE / MSIZE  : 8-bit
     *   TCIE           : transfer complete interrupt enable
     *   PAR            : USART1->DR
     *   FIFO disabled  : direct mode (leave FCR at reset = 0x21, DMDIS=0)
     */
    DMA2_Stream7->CR = (4u << DMA_SxCR_CHSEL_Pos) /* Channel 4         */
                     | DMA_SxCR_DIR_0              /* mem→periph        */
                     | DMA_SxCR_MINC               /* memory increment  */
                     | DMA_SxCR_TCIE;              /* TC interrupt      */

    DMA2_Stream7->PAR  = (uint32_t)(uintptr_t)(&USART1->DR);
    DMA2_Stream7->M0AR = 0u; /* set in dma_kick() */
    DMA2_Stream7->NDTR = 0u;

    /* FIFO: keep in direct mode (default reset value has DMDIS=0) */
    DMA2_Stream7->FCR = 0u;

    /* Enable USART1 DMA TX request */
    USART1->CR3 |= USART_CR3_DMAT;

    /* Enable DMA2 Stream7 IRQ at priority 1 */
    NVIC_SetPriority(DMA2_Stream7_IRQn, 1u);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/* ── uart_dma_write ──────────────────────────────────────────────────── */
size_t uart_dma_write(const char *data, size_t len)
{
    if (!data || len == 0u) return 0u;

    size_t queued = 0u;

    /* Copy bytes into ring buffer, advancing tx_head */
    for (size_t i = 0u; i < len; i++) {
        const uint16_t next = (uint16_t)((tx_head + 1u) % UART_TX_RING_SIZE);
        if (next == tx_tail) break; /* ring full */
        tx_ring[tx_head] = (uint8_t)data[i];
        tx_head = next;
        queued++;
    }

    /* Kick DMA if not already active */
    if (dma_active == 0u) {
        dma_kick();
    }

    return queued;
}

/* ── uart_dma_puts ───────────────────────────────────────────────────── */
void uart_dma_puts(const char *str)
{
    if (!str) return;
    size_t len = 0u;
    while (str[len]) len++;
    uart_dma_write(str, len);
}

/* ── DMA2 Stream7 IRQ handler ────────────────────────────────────────── */
void DMA2_Stream7_IRQHandler(void)
{
    /* Clear transfer-complete flag */
    DMA2->HIFCR = DMA_HIFCR_CTCIF7;

    /* Advance tail by the number of bytes just transferred */
    tx_tail = (uint16_t)((tx_tail + dma_transfer_len) % UART_TX_RING_SIZE);

    dma_active       = 0u;
    dma_transfer_len = 0u;

    /* Kick next segment (handles wrap-around or remaining bytes) */
    dma_kick();
}
