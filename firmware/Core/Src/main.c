#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f407xx.h"
#include "can_frame_queue.h"
#include "can_frame_format.h"
#include "can_signal_profiler.h"
#include "obd2_decoder.h"
#include "uart_dma.h"

/*
 * Project Civic — STM32F407 CAN capture firmware.
 *
 * Transceiver: TJA1051 (NXP) — ONLY supported CAN transceiver.
 * No substitutes, variants, or successors.
 *
 * Features:
 *   1. Adaptive bitrate auto-scan starting at 1 Mbit/s.
 *      Error accumulation (LEC/BOFF) only triggers re-acquisition AFTER a
 *      lock has been established — NOT during the scan phase, so the scan
 *      can advance through candidates without being reset by error bursts
 *      from a wrong-bitrate attempt.
 *   2. Full-drain main loop: ALL queued frames are serialised each iteration,
 *      not just one.  Queue capacity raised to 512 so bursts are absorbed.
 *   3. OBD-II Mode 01 decoder for 30+ PIDs including all engine sensors.
 *   4. Signal profiler: per-ID byte statistics; PROFILE + SIG every 2 s.
 *   5. CAN TX: enabled when PROJECT_CAN_TX_CONFIRMED=1.
 *   6. Adaptive dwell driven by the ISR-side min inter-frame interval, which
 *      is always fresh even when the profiler queue is saturated.
 *
 * Wire protocol to ESP32 (USART1 @ 115200 8N1, PA9=TX, PA10=RX, Discovery P2 header):
 *   STATUS,SCAN,<hz>\r\n
 *   STATUS,LOCK,<hz>\r\n
 *   STATUS,REACQUIRE\r\n
 *   STATUS,DWELL,<dwell_ms>\r\n
 *   <ts>,STD/EXT,<id_nohex>,<dlc>,<rtr>,<payload_hex>\r\n   (no "0x" prefix)
 *   PID,<hex>,<label>,<unit>,<val>\r\n
 *   PROFILE,<id_nohex>,<dlc>,<rate_x10>,<var_mask>,<label>\r\n
 *   SIG,<id_nohex>,<byte>,<last>,<min>,<max>,<rate_x10>,<label>\r\n
 */

/* ── build-time gates ─────────────────────────────────────────────── */
#ifndef PROJECT_CAN_BITRATE_HZ
#define PROJECT_CAN_BITRATE_HZ 0u
#endif
#ifndef PROJECT_TRANSCEIVER_SILENT_CONFIRMED
#define PROJECT_TRANSCEIVER_SILENT_CONFIRMED 0u
#endif
#ifndef PROJECT_CAN_TX_CONFIRMED
#define PROJECT_CAN_TX_CONFIRMED 0u
#endif
#if PROJECT_CAN_BITRATE_HZ > 1000000u
#error "Classical CAN bitrate must not exceed 1 Mbit/s"
#endif

/* ── tuning ───────────────────────────────────────────────────────── */
#define CAN_DWELL_FAST_MS         200u   /* ≤20 ms inter-frame  */
#define CAN_DWELL_MEDIUM_MS       500u   /* ≤100 ms inter-frame */
#define CAN_DWELL_SLOW_MS        1000u   /* >100 ms / no frames */
#define CAN_DWELL_FAST_THRESH_MS   20u
#define CAN_DWELL_MED_THRESH_MS   100u
#define CAN_LOCK_FRAME_COUNT        2u
#define CAN_REACQUIRE_ERROR_MAX     8u
#define HOST_RX_CAPACITY          256u
#define HOST_LINE_CAPACITY        128u

/* ── state ─────────────────────────────────────────────────────────── */
__attribute__((section(".ccmram")))
static can_capture_queue_t capture_queue;
static can_profiler_t      profiler;
static volatile uint32_t   can_error_count;
static volatile uint32_t   system_millis;
static volatile uint32_t   valid_frames_at_rate;
static volatile uint32_t   bitrate_error_streak;
static volatile uint32_t   reacquire_requested;
static volatile uint32_t   can_locked;
static volatile uint32_t   current_bitrate;
static volatile uint32_t   scan_started_ms;
static volatile uint32_t   scan_index;
static volatile uint32_t   host_rx_overflow;
static volatile uint16_t   host_rx_head, host_rx_tail;
static volatile uint8_t    host_rx_ring[HOST_RX_CAPACITY];

static uint32_t current_dwell_ms = CAN_DWELL_SLOW_MS;

/*
 * ISR-side min inter-frame interval tracker.
 * Updated on every RX interrupt — always current even if queue is full.
 * Used by compute_dwell_ms() instead of the profiler's (which only updates
 * when frames are dequeued and may lag behind the real bus rate).
 */
static volatile uint32_t isr_min_interval_ms; /* starts 0 = unknown   */
static volatile uint32_t isr_last_rx_ts_ms;

#if PROJECT_TRANSCEIVER_SILENT_CONFIRMED == 1u
static const uint32_t bitrate_candidates[] = {
    1000000u, 500000u, 250000u, 125000u, 100000u,
    83333u,   50000u,  20000u,  10000u
};
/*
 * 1 Mbit/s first: PCLK1=16 MHz / (prescaler=1 × 16 TQ) = 1,000,000 bit/s.
 * BTR register: TS1 field = 12 → 13 TQ, TS2 field = 1 → 2 TQ,
 * sync-seg = 1 TQ, total = 16 TQ, sample-point = 14/16 = 87.5%.
 * Error = 0%.  All other candidates ≤ 1% error.
 */
#define BITRATE_CANDIDATE_COUNT \
    ((uint32_t)(sizeof(bitrate_candidates)/sizeof(bitrate_candidates[0])))
#endif

/* ══════════════════════════════════════════════════════════════════════
 * P1.1 — 168 MHz PLL clock init
 * ══════════════════════════════════════════════════════════════════════ */
static bool clock_init_168mhz(void)
{
    /* 1. Enable HSE and wait for ready (with timeout) */
    RCC->CR |= RCC_CR_HSEON;
    uint32_t to = 1000000u;
    while (!(RCC->CR & RCC_CR_HSERDY) && --to) { }
    if (!to) return false; /* HSE failed — fall back to HSI 16 MHz */

    /* 2. Set Flash ACR: 5 wait states + PRFTEN + ICEN + DCEN
     *    Required for HCLK > 150 MHz at 3.3V (RM0090 Table 10) */
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN |
                 FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    /* 3. Configure PLL: M=8 (1 MHz VCO in), N=336 (336 MHz VCO),
     *    P=2 (168 MHz SYSCLK), Q=7 (48 MHz for USB), source=HSE */
    RCC->PLLCFGR = (8u  << RCC_PLLCFGR_PLLM_Pos)   /* PLLM = 8  */
                 | (336u << RCC_PLLCFGR_PLLN_Pos)   /* PLLN = 336 */
                 | (0u  << RCC_PLLCFGR_PLLP_Pos)    /* PLLP = /2 (00 = /2) */
                 | RCC_PLLCFGR_PLLSRC_HSE           /* HSE source */
                 | (7u  << RCC_PLLCFGR_PLLQ_Pos);   /* PLLQ = 7 */

    /* 4. Enable PLL, wait for ready */
    RCC->CR |= RCC_CR_PLLON;
    to = 1000000u;
    while (!(RCC->CR & RCC_CR_PLLRDY) && --to) { }
    if (!to) return false;

    /* 5. Set APB prescalers:
     *    AHB /1 → HCLK = 168 MHz
     *    APB2 /2 → PCLK2 = 84 MHz  (bits [15:13] = 100)
     *    APB1 /4 → PCLK1 = 42 MHz  (bits [12:10] = 101)
     *    Switch SYSCLK to PLL (SW bits [1:0] = 10) */
    RCC->CFGR = (RCC->CFGR
                 & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1))
                | RCC_CFGR_HPRE_DIV1       /* AHB /1  */
                | RCC_CFGR_PPRE2_DIV2      /* APB2 /2 */
                | RCC_CFGR_PPRE1_DIV4;     /* APB1 /4 */

    /* 6. Switch SYSCLK to PLL, wait for SWS confirmation */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    to = 1000000u;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL && --to) { }
    if (!to) return false;

    /* 7. Update SystemCoreClock */
    SystemCoreClockUpdate();
    return true;
}

/* ══════════════════════════════════════════════════════════════════════
 * P1.4 — TIM2 microsecond timestamp counter
 * ══════════════════════════════════════════════════════════════════════ */
static void tim2_us_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    /* TIM2 input clock = PCLK1 × 2 (when APB1 prescaler != 1)
     * At 168MHz: PCLK1=42MHz, TIM2 input = 84MHz
     * At 16MHz:  PCLK1=16MHz, TIM2 input = 16MHz (APB1 prescaler=1, so ×1)
     * We want 1 µs tick:
     *   168MHz: PSC = 84-1 = 83
     *   16MHz:  PSC = 16-1 = 15                                             */
    const uint32_t tim2_clk = (SystemCoreClock == 168000000u) ? 84000000u : 16000000u;
    TIM2->PSC = (uint32_t)(tim2_clk / 1000000u) - 1u;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;  /* apply prescaler */
    TIM2->CR1 = TIM_CR1_CEN;
}

/* ══════════════════════════════════════════════════════════════════════
 * SysTick
 * ══════════════════════════════════════════════════════════════════════ */
void SysTick_Handler(void) { system_millis++; }

/* ══════════════════════════════════════════════════════════════════════
 * Everything below compiled only when TJA1051 silent-mode is confirmed
 * ══════════════════════════════════════════════════════════════════════ */
#if PROJECT_TRANSCEIVER_SILENT_CONFIRMED == 1u

/* ── USART1  PA9=TX  PA10=RX  (Discovery P2 header) ─────────────── */
/* Blocking write — kept as fallback for early boot messages before DMA is
 * initialised.  All post-init output goes through uart_dma_puts(). */
static void host_stream_write(const char *data) __attribute__((unused));
static void host_stream_write(const char *data)
{
    while (*data) {
        while ((USART1->SR & USART_SR_TXE) == 0u) { }
        USART1->DR = (uint8_t)*data++;
    }
}

static void host_stream_init(void)
{
    /*
     * USART1: PA9=TX, PA10=RX, AF7, 115200 baud at 16 MHz HSI.
     *
     * On STM32F407G-DISC1 Discovery board:
     *   PA2/PA3 (USART2) are NOT on any header — they route to the
     *   ST-Link MCU only and cannot be accessed without soldering to IC legs.
     *   PA9/PA10 (USART1) ARE on header P2:
     *     P2 right column — PA9  = USART1_TX  → ESP32 GPIO16 (RX2)
     *     P2 right column — PA10 = USART1_RX  ← ESP32 GPIO17 (TX2)
     *
     * USART1 is on APB2. At 16 MHz HSI with no APB2 prescaler,
     * APB2 = 16 MHz, so BRR = 139 gives exactly 115200 baud.
     */
    SystemCoreClockUpdate();
    if (SystemCoreClock != 168000000u && SystemCoreClock != 16000000u) {
        can_error_count++; return;
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   /* USART1 on APB2 */
    (void)RCC->APB2ENR;
    /* PA9=TX: MODER bits [19:18]=10(AF), PA10=RX: bits [21:20]=10(AF) */
    GPIOA->MODER   = (GPIOA->MODER   & ~((3u<<18)|(3u<<20))) | (2u<<18)|(2u<<20);
    GPIOA->OTYPER  &= ~((1u<<9)|(1u<<10));
    GPIOA->PUPDR   &= ~((3u<<18)|(3u<<20));
    GPIOA->OSPEEDR  = (GPIOA->OSPEEDR & ~((3u<<18)|(3u<<20))) | (2u<<18)|(2u<<20);
    /* AF7: PA9 → AFR[1] bits [7:4], PA10 → AFR[1] bits [11:8] */
    GPIOA->AFR[1]   = (GPIOA->AFR[1]  & ~((15u<<4)|(15u<<8))) | (7u<<4)|(7u<<8);
    USART1->CR1 = 0u;
    /* At PCLK2=84 MHz: BRR=16 → 5.25 Mbit/s (0% error)
     * At PCLK2=16 MHz: BRR=139 → 115200 baud (fallback) */
    const uint32_t pclk2 = (SystemCoreClock == 168000000u) ? 84000000u : 16000000u;
    USART1->BRR = (uint32_t)(pclk2 / 5250000u);
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;
    USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_SR_RXNE) != 0u) {
        const uint8_t ch = (uint8_t)USART1->DR;
        const uint16_t nxt = (uint16_t)((host_rx_head + 1u) % HOST_RX_CAPACITY);
        if (nxt != host_rx_tail) { host_rx_ring[host_rx_head] = ch; host_rx_head = nxt; }
        else { host_rx_overflow++; }
    }
}

static bool host_rx_pop(uint8_t *out)
{
    if (host_rx_tail == host_rx_head) return false;
    *out = host_rx_ring[host_rx_tail];
    host_rx_tail = (uint16_t)((host_rx_tail + 1u) % HOST_RX_CAPACITY);
    return true;
}

/* ── CAN1 GPIO (TJA1051 wiring) ─────────────────────────────────── */
static void gpio_can1_init(void)
{
    /*
     * TJA1051 (NXP) wiring:
     *   PB8 = CAN1_RX ← TJA1051 TXD  (AF9)
     *   PB9 = CAN1_TX → TJA1051 RXD  (AF9)
     *   TJA1051 STB tied to GND  → normal/active mode always
     *   TJA1051 CANH/CANL → OBD-II pins 6 & 14
     * Silent monitoring uses bxCAN SILM bit — no dominant bits on bus.
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;
    GPIOB->MODER   = (GPIOB->MODER   & ~((3u<<16)|(3u<<18))) | (2u<<16)|(2u<<18);
    GPIOB->OTYPER  &= ~((1u<<8)|(1u<<9));
    GPIOB->PUPDR   &= ~((3u<<16)|(3u<<18));
    GPIOB->OSPEEDR  = (GPIOB->OSPEEDR & ~((3u<<16)|(3u<<18))) | (3u<<16)|(3u<<18);
    GPIOB->AFR[1]   = (GPIOB->AFR[1]  & ~((15u<<0)|(15u<<4))) | (9u<<0)|(9u<<4);
}

/* ── bxCAN bitrate / mode ───────────────────────────────────────── */
static bool can1_set_bitrate(uint32_t bitrate, bool silent)
{
    /* PCLK1 depends on SystemCoreClock:
     * At 168MHz: APB1 /4 → PCLK1 = 42MHz
     * At  16MHz: APB1 /1 → PCLK1 = 16MHz                                   */
    const uint32_t pclk1 = (SystemCoreClock == 168000000u) ? 42000000u : 16000000u;
    const uint32_t tq = 16u;
    if (!bitrate) return false;
    const uint32_t denom = bitrate * tq;
    const uint32_t presc = (pclk1 + denom/2u) / denom;
    if (presc < 1u || presc > 1024u) return false;
    const uint32_t actual = pclk1 / (presc * tq);
    const uint32_t delta  = actual > bitrate ? actual-bitrate : bitrate-actual;
    if ((delta * 100u) > (bitrate * 1u)) return false; /* >1% error */

    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN; (void)RCC->APB1ENR;
    CAN1->MCR |= CAN_MCR_INRQ;
    uint32_t to = 1000000u;
    while (!(CAN1->MSR & CAN_MSR_INAK) && --to) { }
    if (!to) { can_error_count++; return false; }

    CAN1->MCR |= CAN_MCR_ABOM;
    /* TS1 field=12→13 TQ, TS2 field=1→2 TQ, SJW=0→1 TQ, total=16 TQ */
    CAN1->BTR = ((presc-1u) & 0x3ffu) | (12u<<16) | (1u<<20)
                | (silent ? CAN_BTR_SILM : 0u);

    /* Filter bank 0: 32-bit mask, pass everything (STD and EXT) */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~1u; CAN1->FM1R &= ~1u; CAN1->FS1R |= 1u; CAN1->FFA1R &= ~1u;
    CAN1->sFilterRegister[0].FR1 = 0u;
    CAN1->sFilterRegister[0].FR2 = 0u;
    CAN1->FA1R |= 1u;
    CAN1->FMR &= ~CAN_FMR_FINIT;

    CAN1->MCR &= ~CAN_MCR_INRQ;
    to = 1000000u;
    while ((CAN1->MSR & CAN_MSR_INAK) && --to) { }
    if (!to) { can_error_count++; return false; }

    CAN1->IER |= CAN_IER_FMPIE0 | CAN_IER_ERRIE | CAN_IER_BOFIE;
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_SCE_IRQn);

    current_bitrate      = actual;
    valid_frames_at_rate = 0u;
    bitrate_error_streak = 0u;
    scan_started_ms      = system_millis;
    isr_last_rx_ts_ms    = 0u;
    isr_min_interval_ms  = 0u;
    return true;
}

static void can1_start_passive(void)
{
    SystemCoreClockUpdate();
    if (SystemCoreClock != 168000000u && SystemCoreClock != 16000000u) {
        can_error_count++; return;
    }
    can_locked = 0u; scan_index = 0u;
    if (can1_set_bitrate(bitrate_candidates[0], true)) {
        char s[48];
        (void)snprintf(s, sizeof s, "STATUS,SCAN,%lu\r\n", (unsigned long)current_bitrate);
        uart_dma_puts(s);
    }
}

/* ── Adaptive dwell (ISR-driven, always current) ───────────────── */
static uint32_t compute_dwell_ms(void)
{
    /*
     * Use the ISR-side minimum interval — it is updated on every received
     * frame regardless of whether the queue was full.  The profiler's
     * min_interval is only updated when frames are dequeued (may lag).
     */
    const uint32_t mi = isr_min_interval_ms;
    if (mi == 0u)                         return CAN_DWELL_SLOW_MS;
    if (mi <= CAN_DWELL_FAST_THRESH_MS)   return CAN_DWELL_FAST_MS;
    if (mi <= CAN_DWELL_MED_THRESH_MS)    return CAN_DWELL_MEDIUM_MS;
    return CAN_DWELL_SLOW_MS;
}

/* ── Bitrate scan state machine ─────────────────────────────────── */
static void can1_scan_step(void)
{
    if (can_locked) return;

    if (valid_frames_at_rate >= CAN_LOCK_FRAME_COUNT) {
        can_locked = 1u;
        char s[64];
        (void)snprintf(s, sizeof s, "STATUS,LOCK,%lu\r\n", (unsigned long)current_bitrate);
        uart_dma_puts(s);
        (void)snprintf(s, sizeof s, "STATUS,DWELL,%lu\r\n", (unsigned long)current_dwell_ms);
        uart_dma_puts(s);
        return;
    }

    /* Extended patience when exactly 1 frame seen */
    if (valid_frames_at_rate == 1u &&
        (uint32_t)(system_millis - scan_started_ms) < current_dwell_ms * 3u) return;

    if ((uint32_t)(system_millis - scan_started_ms) < current_dwell_ms) return;

    /* Advance to next candidate */
    scan_index = (scan_index + 1u) % BITRATE_CANDIDATE_COUNT;
    uint32_t tries = 0u;
    while (tries < BITRATE_CANDIDATE_COUNT &&
           !can1_set_bitrate(bitrate_candidates[scan_index], true)) {
        scan_index = (scan_index + 1u) % BITRATE_CANDIDATE_COUNT;
        tries++;
    }
    if (tries < BITRATE_CANDIDATE_COUNT) {
        char s[48];
        (void)snprintf(s, sizeof s, "STATUS,SCAN,%lu\r\n", (unsigned long)current_bitrate);
        uart_dma_puts(s);
    }
}

/* ── TX support ─────────────────────────────────────────────────── */
#if PROJECT_CAN_TX_CONFIRMED == 1u
static bool can1_set_controller_silent(bool silent)
{
    CAN1->MCR |= CAN_MCR_INRQ;
    uint32_t to = 1000000u;
    while (!(CAN1->MSR & CAN_MSR_INAK) && --to) { }
    if (!to) return false;
    if (silent) CAN1->BTR |= CAN_BTR_SILM; else CAN1->BTR &= ~CAN_BTR_SILM;
    CAN1->MCR &= ~CAN_MCR_INRQ;
    to = 1000000u;
    while ((CAN1->MSR & CAN_MSR_INAK) && --to) { }
    return to != 0u;
}
static int hex_nibble(char c)
{
    if (c>='0'&&c<='9') return c-'0';
    if (c>='a'&&c<='f') return c-'a'+10;
    if (c>='A'&&c<='F') return c-'A'+10;
    return -1;
}
#endif

/* ── Host command parser ─────────────────────────────────────────── */
static void host_command(char *lb)
{
    char *cmd=strtok(lb,","); if(!cmd||strcmp(cmd,"TX")!=0) return;
    char *fmt=strtok(NULL,","), *id_t=strtok(NULL,","),
         *rtr_t=strtok(NULL,","), *dlc_t=strtok(NULL,","),
         *hex_t=strtok(NULL,",");
    if (!fmt||!id_t||!rtr_t||!dlc_t||!hex_t||
        (strcmp(fmt,"S")!=0&&strcmp(fmt,"E")!=0)) {
        uart_dma_puts("TX,ERR,BAD_FORMAT\r\n"); return;
    }
#if PROJECT_CAN_TX_CONFIRMED != 1u
    (void)fmt;(void)id_t;(void)rtr_t;(void)dlc_t;(void)hex_t;
    uart_dma_puts("TX,ERR,TX_HARDWARE_NOT_CONFIRMED\r\n");
#else
    if (!can_locked) { uart_dma_puts("TX,ERR,NO_BITRATE_LOCK\r\n"); return; }
    const uint32_t id  = (uint32_t)strtoul(id_t,  NULL, 16);
    const uint32_t rtr = (uint32_t)strtoul(rtr_t, NULL, 10);
    const uint32_t dlc = (uint32_t)strtoul(dlc_t, NULL, 10);
    const bool ext = strcmp(fmt,"E")==0;
    if (rtr>1u||dlc>8u||(ext?id>0x1fffffffu:id>0x7ffu)||
        (rtr?strcmp(hex_t,"-")!=0:strlen(hex_t)!=dlc*2u)) {
        uart_dma_puts("TX,ERR,BAD_FIELDS\r\n"); return;
    }
    uint8_t data[8]={0};
    for (uint32_t i=0;i<dlc&&!rtr;i++) {
        int hi=hex_nibble(hex_t[i*2]), lo=hex_nibble(hex_t[i*2+1]);
        if (hi<0||lo<0) { uart_dma_puts("TX,ERR,BAD_HEX\r\n"); return; }
        data[i]=(uint8_t)((hi<<4)|lo);
    }
    if (!can1_set_controller_silent(false)) {
        uart_dma_puts("TX,ERR,CAN_MODE_SWITCH_FAILED\r\n"); return;
    }
    uint32_t tsr=CAN1->TSR, mb=0u;
    if (!(tsr&(1u<<26))) {
        if (tsr&(1u<<27)) mb=1u;
        else if (tsr&(1u<<28)) mb=2u;
        else { uart_dma_puts("TX,ERR,MAILBOX_BUSY\r\n"); return; }
    }
    CAN1->sTxMailBox[mb].TIR  = 0u;
    CAN1->sTxMailBox[mb].TDTR = dlc;
    CAN1->sTxMailBox[mb].TDLR = (uint32_t)data[0]|((uint32_t)data[1]<<8)|
                                  ((uint32_t)data[2]<<16)|((uint32_t)data[3]<<24);
    CAN1->sTxMailBox[mb].TDHR = (uint32_t)data[4]|((uint32_t)data[5]<<8)|
                                  ((uint32_t)data[6]<<16)|((uint32_t)data[7]<<24);
    uint32_t tir = ext?((id&0x1fffffffu)<<3)|(1u<<2):((id&0x7ffu)<<21);
    if (rtr) tir |= 1u<<1;
    CAN1->sTxMailBox[mb].TIR = tir | 1u;
    uart_dma_puts("TX,QUEUED\r\n");
#endif
}

static void host_command_poll(void)
{
    static char lb[HOST_LINE_CAPACITY];
    static uint32_t used = 0u;
    uint8_t ch;
    while (host_rx_pop(&ch)) {
        if (ch=='\r') continue;
        if (ch=='\n') { lb[used]='\0'; if(used) host_command(lb); used=0u; }
        else if (used+1u<sizeof lb) lb[used++]=(char)ch;
        else { used=0u; uart_dma_puts("TX,ERR,LINE_TOO_LONG\r\n"); }
    }
}

#endif /* PROJECT_TRANSCEIVER_SILENT_CONFIRMED */

/* ══════════════════════════════════════════════════════════════════════
 * CAN interrupt handlers
 * ══════════════════════════════════════════════════════════════════════ */
void CAN1_RX0_IRQHandler(void)
{
    while (CAN1->RF0R & CAN_RF0R_FMP0) {
        const uint32_t rir  = CAN1->sFIFOMailBox[0].RIR;
        const uint32_t rdtr = CAN1->sFIFOMailBox[0].RDTR;
        const uint32_t low  = CAN1->sFIFOMailBox[0].RDLR;
        const uint32_t high = CAN1->sFIFOMailBox[0].RDHR;

        can_capture_frame_t f = {0};
        f.timestamp_ms = system_millis;
        f.timestamp_us = TIM2->CNT;
        f.extended = (rir & (1u<<2)) ? 1u : 0u;
        f.remote   = (rir & (1u<<1)) ? 1u : 0u;
        f.identifier = f.extended ? ((rir>>3)&0x1fffffffu) : ((rir>>21)&0x7ffu);
        f.dlc = (uint8_t)(rdtr & 0x0fu); if (f.dlc>8u) f.dlc=8u;
        if (!f.remote) {
            for (uint32_t i=0;i<f.dlc&&i<4;i++) f.data[i]=(uint8_t)(low >>(i*8));
            for (uint32_t i=4;i<f.dlc;i++)       f.data[i]=(uint8_t)(high>>((i-4)*8));
        }

        /* ISR-side min interval — always updated even if queue overflows */
        const uint32_t now_ms = system_millis;
        if (isr_last_rx_ts_ms != 0u) {
            const uint32_t intv = now_ms - isr_last_rx_ts_ms;
            if (intv > 0u && (isr_min_interval_ms == 0u || intv < isr_min_interval_ms))
                isr_min_interval_ms = intv;
        }
        isr_last_rx_ts_ms = now_ms;

        valid_frames_at_rate++;
        bitrate_error_streak = 0u;
        if (!can_capture_queue_push_isr(&capture_queue, &f)) can_error_count++;
        CAN1->RF0R = CAN_RF0R_RFOM0;
    }
    if (CAN1->RF0R & CAN_RF0R_FOVR0) { can_error_count++; CAN1->RF0R = CAN_RF0R_FOVR0; }
}

void CAN1_SCE_IRQHandler(void)
{
    const uint32_t esr = CAN1->ESR;
    if (esr & (CAN_ESR_BOFF|CAN_ESR_EPVF|CAN_ESR_EWGF)) can_error_count++;
    if (esr & CAN_ESR_LEC) {
        bitrate_error_streak++;
        if (bitrate_error_streak >= CAN_REACQUIRE_ERROR_MAX && can_locked)
            reacquire_requested = 1u; /* only reacquire AFTER a lock */
    }
    if ((esr & CAN_ESR_BOFF) && can_locked) /* bus-off only after lock */
        reacquire_requested = 1u;
    CAN1->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI;
}

/* ══════════════════════════════════════════════════════════════════════
 * Public API
 * ══════════════════════════════════════════════════════════════════════ */
bool     project_capture_pop(can_capture_frame_t *out) { return can_capture_queue_pop(&capture_queue, out); }
uint32_t project_capture_dropped(void)  { return capture_queue.dropped; }
uint32_t project_capture_errors(void)   { return can_error_count; }
uint32_t project_capture_millis(void)   { return system_millis; }

/* ══════════════════════════════════════════════════════════════════════
 * main()
 * ══════════════════════════════════════════════════════════════════════ */
int main(void)
{
    clock_init_168mhz(); /* P1.1 — attempt 168 MHz; falls back to 16 MHz HSI on failure */
    can_capture_queue_init(&capture_queue);
    can_profiler_init(&profiler);
    if (SysTick_Config(SystemCoreClock / 1000u) != 0u) for (;;) { }
    tim2_us_init(); /* P1.4 — 1 µs free-running counter on TIM2 */

#if PROJECT_TRANSCEIVER_SILENT_CONFIRMED == 1u
    host_stream_init();
    uart_dma_init(); /* P1.3 — DMA TX for USART1 */
    gpio_can1_init();
    can1_start_passive();

    static uint32_t last_dwell_report_ms = 0u;

    for (;;) {
        host_command_poll();

        /* Re-acquisition: only after a lock (not during scan) */
        if (reacquire_requested) {
            reacquire_requested  = 0u;
            can_locked           = 0u;
            scan_index           = 0u;
            valid_frames_at_rate = 0u;
            isr_last_rx_ts_ms    = 0u;
            isr_min_interval_ms  = 0u;
            can_profiler_init(&profiler);
            if (can1_set_bitrate(bitrate_candidates[0], true))
                uart_dma_puts("STATUS,REACQUIRE\r\n");
        }

        /* Adaptive dwell update */
        {
            const uint32_t nd = compute_dwell_ms();
            if (nd != current_dwell_ms) {
                current_dwell_ms = nd;
                if ((system_millis - last_dwell_report_ms) >= 5000u) {
                    last_dwell_report_ms = system_millis;
                    char s[48];
                    (void)snprintf(s, sizeof s, "STATUS,DWELL,%lu\r\n",
                                   (unsigned long)current_dwell_ms);
                    uart_dma_puts(s);
                }
            }
        }

        can1_scan_step();

        /*
         * Drain ALL available frames per iteration, not just one.
         * This maximises throughput when the UART is keeping up, and
         * ensures the profiler sees data as fast as possible.
         * At 115200 baud / ~50 bytes/frame the sustainable rate is
         * ~178 frames/s; excess frames are silently dropped by the queue
         * ISR and counted in capture_queue.dropped.
         */
        {
            can_capture_frame_t frame;
            while (can_capture_queue_pop(&capture_queue, &frame)) {
                char csv[72];
                if (can_capture_frame_to_csv(&frame, csv, sizeof csv))
                    uart_dma_puts(csv);

                char pid[OBD2_LINE_CAPACITY];
                if (obd2_decode(&frame, pid, sizeof pid))
                    uart_dma_puts(pid);

                can_profiler_update(&profiler, &frame);
            }
        }

        /* Emit PROFILE + SIG lines every 2 s */
        can_profiler_emit(&profiler, system_millis, uart_dma_puts);
    }

#else
    for (;;) { __WFI(); }
#endif
}
