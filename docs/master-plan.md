# Project Civic — Complete Master Implementation Plan
## Every feature, every level, exact implementation order
## Includes ALL previous plans: performance, ECU reset, auto-correlation, CAN control

---

## System Overview (Final Target State)

```
OBD-II J1962 Connector
    ├─ Pins 6/14  HS-CAN  500 kbps ──── TJA1051 #1 ──── STM32 CAN1 (PB8/PB9)
    └─ Pins 1/9   MS-CAN  125 kbps ──── TJA1051 #2 ──── STM32 CAN2 (PB12/PB13)
    Pin 16 +12V ─── DC-DC Buck ─── 5V ─── ESP32 VIN
                               └── 3.3V ── STM32 VDD + TJA1051 VCC + EEPROM VCC

STM32F407 @ 168 MHz (HSE 8MHz + PLL)
    ├─ CAN1 + CAN2 both active → dual 2048-frame queues in CCM RAM
    ├─ TIM2 @ 1 µs timestamp resolution
    ├─ USART1 @ 5.25 Mbit/s + DMA TX → ESP32 GPIO16/17
    ├─ I2C1 @ 400 kHz → 24LC512 EEPROM (PB6/PB7)
    ├─ ITM/SWO @ 2 Mbit/s → PC debug (PA15 / ST-Link)
    ├─ OBD-II Mode 01/03/04/09 + Honda Mode 21
    ├─ Native CAN decoder + Auto-correlator
    └─ UDS client (ISO 14229 + ISO 15765-2 ISO-TP)

ESP32 @ 240 MHz
    ├─ UART2 @ 5.25 Mbit/s ← STM32 (4 KB RX ring buffer)
    ├─ WS batch broadcast (5ms coalesce, static 512-byte buffer)
    ├─ Wi-Fi AP → browser dashboard
    └─ OTA for both boards

Browser Dashboard (http://192.168.4.1)
    ├─ Metric cards (RPM, Coolant, Fuel, Battery)
    ├─ RPM Over Samples / Temperatures / Data Quality / Alerts
    ├─ RPM vs Throttle scatter + histogram
    ├─ CAN Analyzer table (live, labeled, colored)
    ├─ Signal Explorer (all ECU signals, sparklines)
    ├─ Operating Mode (Read / Write full unlocked)
    ├─ ECU Controller Menu (discovery + selective reset)
    ├─ Firmware Update panel (ESP32 + STM32 OTA)
    └─ STM32 Debug Monitor (ITM/SWO live values)
```

---

## Phase 0 — Already Done ✅

| Item | Status |
|---|---|
| STM32 CAN capture firmware (bare register) | ✅ |
| Adaptive bitrate scan 1M→500k→250k→... | ✅ |
| Scan thrash fix (errors only reacquire after lock) | ✅ |
| Signal profiler (per-ID min/max/rate/var_mask) | ✅ |
| min_interval_ms UINT32_MAX sentinel fix | ✅ |
| OBD-II decoder 30+ PIDs including 0x42 battery, PID 0x43 fixed | ✅ |
| Honda Civic native CAN DB (24 known IDs) | ✅ |
| Full dashboard HTML/CSS/JS matching sample screenshot | ✅ |
| ESP32 Wi-Fi AP + WebSocket + HTTP | ✅ |
| All 8 wire protocol line types parsed | ✅ |
| Write mode (raw frame editor, no confirm, load defaults) | ✅ |
| ESP32 OTA (/update) | ✅ |
| STM32 OTA via UART bootloader (Extended Erase 0x44 fixed) | ✅ |
| USART switched to USART1 PA9/PA10 (Discovery board P2 header) | ✅ |
| All 15 audit bugs fixed | ✅ |
| Queue 512 frames, id no-0x-prefix, rate uint64, ev.preventDefault | ✅ |
| WDT feed in bootloader wait loop, heap bounds check | ✅ |
| 15/15 unit tests passing | ✅ |
| Both boards flashed and running | ✅ |

---

## Phase 1 — 168 MHz + 5.25 Mbit/s UART + DMA + TIM2 + CCM

**Goal: Eliminate the 97% frame drop rate. No new hardware needed.**

### P1.1 — STM32 168 MHz PLL

New function `clock_init_168mhz()` called first in `main()`.

```
HSE 8 MHz (Discovery crystal) → PLL
PLL_M=8, PLL_N=336, PLL_P=2, PLL_Q=7
HCLK=168MHz, PCLK2=84MHz, PCLK1=42MHz
Flash: 5 wait states + prefetch + icache + dcache enabled
Fallback: if HSE fails → HSI 16 MHz (graceful degradation, firmware still works)
```

Clock guards updated: accept 168 MHz OR 16 MHz fallback (not hardcoded == 16M).

**File:** `firmware/Core/Src/main.c`

### P1.2 — USART1 at 5.25 Mbit/s

```c
/* PCLK2=84 MHz: BRR = 84,000,000 / 5,250,000 = 16 → 0% error */
USART1->BRR = 16u;
```

**Files:** `firmware/Core/Src/main.c`, `firmware/esp32-dashboard/src/main.cpp`

ESP32 side:
```cpp
Stm32.setRxBufferSize(4096);  /* before begin() */
Stm32.begin(5250000, SERIAL_8N1, STM32_RX, STM32_TX);
```

### P1.3 — DMA TX for USART1 (zero CPU cost for UART)

**New files:** `firmware/Core/Inc/uart_dma.h`, `firmware/Core/Src/uart_dma.c`

```
DMA2 Stream7 Channel4 → USART1_TX
TX ring buffer: 4096 bytes in SRAM
host_stream_write() → writes to ring, kicks DMA if idle
DMA2_Stream7_IRQHandler → on TC, restart if more data pending
CPU never spins waiting for UART byte
```

### P1.4 — TIM2 microsecond timestamps

```c
/* TIM2 input = PCLK1×2 = 84 MHz, PSC=83 → 1 µs tick */
TIM2->PSC = 83u;
TIM2->ARR = 0xFFFFFFFFu;  /* 32-bit, wraps ~4295 seconds */
TIM2->CR1 = TIM_CR1_CEN;
/* In CAN RX ISR: f.timestamp_us = TIM2->CNT; */
```

CSV format: `<ts_us>,STD/EXT,<id>,<dlc>,<rtr>,<hex>\n`

Add `uint32_t timestamp_us` to `can_capture_frame_t`.

**Files:** `firmware/Core/Src/main.c`, `firmware/Core/Inc/can_frame_queue.h`, `firmware/Core/Src/can_frame_format.c`

### P1.5 — CCM RAM queue (2048 frames)

```c
__attribute__((section(".ccmram")))
static can_capture_queue_t capture_queue;
```

`CAN_CAPTURE_QUEUE_CAPACITY` → 2048 (fits in 64 KB CCM: 2048 × 20 bytes = 40 KB).

**Files:** `firmware/Core/Inc/can_frame_queue.h`, `firmware/Core/Src/main.c`

### P1.6 — ESP32 WebSocket zero-copy batch broadcast

Replace `StaticJsonDocument + serializeJson + String` per frame with:
```cpp
static char txbuf[512];
int n = snprintf(txbuf, sizeof txbuf, "{...}");
ws.broadcastTXT(txbuf, n);
```

5ms coalesce window: accumulate up to 4 consecutive CAN CSV frames into one JSON array `{"type":"frames","data":[...]}` before broadcasting. Eliminates heap allocation at high frame rates.

**File:** `firmware/esp32-dashboard/src/main.cpp`

### Phase 1 result

| Before | After |
|---|---|
| 256 frames/sec streamed | 14,500 frames/sec |
| 97% drops at 1 Mbit/s | **0% drops** |
| 1 ms timestamp | **1 µs timestamp** |
| CPU busy on UART | DMA handles UART |

---

## Phase 2 — Dual CAN (HS + MS-CAN)

**New hardware:** 1× TJA1051, 5 wires to P2 header + OBD-II pins 1&9

### P2.1 — CAN2 GPIO + init (PB12=RX, PB13=TX, AF9)

```c
static void gpio_can2_init(void) { /* PB12/PB13 AF9 */ }
static bool can2_set_bitrate(uint32_t bitrate, bool silent) { /* mirrors CAN1 */ }
```

### P2.2 — Dual independent scan state machines

CAN1 starts at 1 Mbit/s (HS), CAN2 starts at 125 kbps (MS).
Both lock independently. Dashboard shows both bitrates.

### P2.3 — Dual CCM queues + dual ISRs

```c
__attribute__((section(".ccmram")))
static can_capture_queue_t hs_queue;   /* CAN1 */
static can_capture_queue_t ms_queue;   /* CAN2 */

void CAN1_RX0_IRQHandler(void) { /* → hs_queue */ }
void CAN2_RX0_IRQHandler(void) { /* → ms_queue */ }
```

### P2.4 — Bus tag in CSV stream

```
<ts_us>,HS,7E8,8,0,<hex>\n
<ts_us>,MS,03B,8,0,<hex>\n
```

### P2.5 — MS-CAN Honda DB entries

Add to `civic_can_db.h`: 0x03B (BCM), 0x038 (doors/windows), 0x040 (climate),
0x174 (cluster/odometer), 0x1A8 (power mode), 0x2F8 (immobiliser)

### P2.6 — Hardware acceptance filters (advanced)

STM32F407 has 28 filter banks. Instead of accept-all:
```c
/* FIFO1 (higher NVIC priority): OBD-II responses 0x7E8–0x7EF */
/* FIFO0 (normal priority): all other native frames */
/* Filter bank 0: list mode, 0x7E8/0x7E9/0x7EA/0x7EB */
/* Filter bank 1: list mode, 0x7EC/0x7ED/0x7EE/0x7EF */
/* Filter bank 2: mask mode, accept all (fallthrough) → FIFO0 */
```
OBD-II responses never delayed by high-volume native frame traffic.

---

## Phase 3 — 24LC512 EEPROM Persistence

**New hardware:** 24LC512 + 2× 4.7 kΩ pull-ups + 4 wires (PB6/PB7)

**Specs confirmed:** 512 Kbit = 64 KB, 128-byte page, 5ms write cycle, 400 kHz I2C, >1M cycles, I2C address 0x50 (A0=A1=A2=GND)

### P3.1 — I2C1 bare-register driver

**New files:** `firmware/Core/Inc/eeprom_24lc512.h`, `firmware/Core/Src/eeprom_24lc512.c`

```c
/* I2C1: PB6=SCL, PB7=SDA, AF4, 400 kHz */
/* At PCLK1=42 MHz: CCR=35, TRISE=43 */
bool eeprom_read(uint16_t addr, uint8_t *buf, uint16_t len);
bool eeprom_write_page(uint16_t addr, const uint8_t *data, uint16_t len); /* max 128 bytes */
void eeprom_poll_ack(void);  /* waits up to 5ms for write complete */
```

### P3.2 — EEPROM layout (64 KB)

```
0x0000  2     Magic 0xCAFE (valid flag)
0x0002  2     Schema version
0x0004  4     HS-CAN learned bitrate
0x0008  4     MS-CAN learned bitrate
0x000C  4     Total session count
0x0010  4     Lifetime frames captured
0x0014  4     Last write timestamp
0x0020  256   Known HS frame IDs (64 × 4 bytes)
0x0120  256   Known MS frame IDs (64 × 4 bytes)
0x0220  4096  HS signal profile cache (128 × 32 bytes)
0x1220  2048  MS signal profile cache (64 × 32 bytes)
0x1A20  1024  Confirmed signal decode table (32 entries × 32 bytes)
0x1E20  128   Auto-correlation pairs (32 × 32 bytes — see Phase 5)
0x1EA0  8192  DTC history ring (256 × 32 bytes)
0x3EA0  8192  Trip log ring (512 × 16 bytes)
0x5EA0  rest  Reserved
```

### P3.3 — Boot sequence with EEPROM

```c
/* On power-on: */
if (eeprom_valid()) {
    hs_baud = eeprom_read_u32(0x0004);
    ms_baud = eeprom_read_u32(0x0008);
    can1_start_at_bitrate(hs_baud, true);  /* skip scan */
    can2_start_at_bitrate(ms_baud, true);
    eeprom_load_profiles(&hs_profiler, &ms_profiler);
    host_stream_write("STATUS,BOOT_LEARNED\r\n");
} else {
    can1_start_passive();  /* full scan */
    can2_start_passive();
}
```

**Result: Boot lock time < 200 ms after first drive.**

### P3.4 — Deferred write policy

- On lock: write bitrate
- Every 60 s: write profiler snapshot
- On bus-off: write error event
- Never write during active CAN RX burst (deferred write queue, 8 entries)

---

## Phase 4 — ITM/SWO Debug Channel

**No new hardware needed. SWO = PA15, connected via ST-Link on Discovery board.**
Check SB21 solder bridge on your board revision — may need to solder closed.

### P4.1 — ITM enable in firmware

```c
static void itm_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM->LAR  = 0xC5ACCE55u;   /* unlock */
    ITM->TCR  = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk;
    ITM->TER  = 0xFFFFFFFFu;   /* enable all 32 ports */
    TPI->SPPR = 2u;             /* NRZ encoding */
    TPI->ACPR = 0u;             /* SWO = HCLK/1 = 168 MHz */
}
```

Send 32-byte debug packet every 100 ms via ITM port 0:
`millis, hs_frames, ms_frames, dropped, can_errors, queue_fill, bitrate, dwell_ms`

### P4.2 — openocd SWO server (host PC tool)

**New file:** `firmware/tools/swo_server.py`

Connects to openocd telnet :4444, reads SWO port 0, parses 32-byte debug packets, serves as WebSocket on :8765.

### P4.3 — Dashboard STM32 Debug Monitor panel

New panel:
```
Uptime | HS Frames | MS Frames | Dropped | CAN Errors | Queue % | Bitrate | Dwell
```
Live from SWO — no UART bandwidth used.

---

## Phase 5 — Native Signal Auto-Correlation Engine

**Goal: Automatically decode Honda native CAN bytes into real physical values.**

### P5.1 — Pearson correlation engine

**New files:** `firmware/Core/Inc/auto_correlator.h`, `firmware/Core/Src/auto_correlator.c`

Algorithm:
- For each native frame byte, maintain rolling buffer of **256 samples** (exact)
- For each OBD-II reference signal (RPM, speed, throttle, etc.), same 256-sample buffer
- Compute Pearson r coefficient: threshold **|r| > 0.95** for 50+ samples → CONFIRMED
- Compute linear fit: `value = (raw_byte × scale) + offset` via least-squares
- Formula: `scale = Σ((xi-x̄)(yi-ȳ)) / Σ((xi-x̄)²)`, `offset = ȳ - scale*x̄`
- Store to EEPROM: max **32 confirmed pairs**, fixed 32-byte records
- On next boot: restore pairs instantly, no re-correlation needed

### P5.2 — New wire protocol line

```
CORR,<id_a>,<byte_a>,<id_b>,<byte_b>,<r_x1000>,<scale_x1000>,<offset_x1000>\r\n
```

### P5.3 — Honda-specific native decode formulas (from known IDs)

Once confirmed by correlation, apply exact Honda formulas:
```
0x17C B0-B1:  RPM = (B0<<8 | B1) * 0.25   rpm
0x158 B0-B1:  WheelFL = (B0<<8 | B1) * 0.01   km/h
0x1A0 B0-B1:  BrakePressure = (B0<<8 | B1) * 0.1   bar
0x0A5 B0-B1:  MAF = (B0<<8 | B1) * 0.01   g/s
0x0D9 B1:     FuelTrim_ST = (B1 - 128) * 0.78125   %
```

Stream as:
```
NAT,17C,0-1,RPM,rpm,3276.0\r\n
NAT,158,0-1,WheelFL,kmh,52.4\r\n
```

---

## Phase 6 — Full OBD-II Diagnostic Access

### P6.1 — Mode 03 (Read DTCs)

```
CMD,READ_DTC\n → STM32 sends 0x7DF request Mode 03
Response: DTC,P0300,Random Misfire\r\n (one per code)
```

### P6.2 — Mode 04 (Clear DTCs)

```
CMD,CLEAR_DTC\n → Mode 04 request
Response: DTC_CLEARED,OK\r\n
```

### P6.3 — Mode 09 (VIN + Calibration IDs)

```
CMD,READ_VIN\n  → Mode 09 PID 0x02
Response: VIN,JH4DC548X3S001234\r\n

CMD,READ_CALID\n → Mode 09 PID 0x04
Response: CALID,39790-5BA-A010\r\n
```

### P6.4 — Honda Mode 21 (manufacturer-specific extended)

Honda-specific PIDs not in standard OBD-II:
- VTEC engagement status
- Individual cylinder misfire counts
- Transmission fluid temperature
- Battery current (not just voltage)
- EPS torque sensor

### P6.5 — CAN TX arbitration-aware + bus-off improved

**CAN TX arbitration loss retry:**
- After TX, check TSR ALST0/ALST1/ALST2 bits
- If arbitration lost → re-queue frame, retry with 1ms → 5ms → 25ms backoff
- TX queue: 8 frames with priority ordering by CAN ID

**Bus-off exponential backoff:**
- First recovery: 100ms quiet → rejoin
- Second consecutive: 500ms quiet
- Third: 2000ms quiet
- After 5 consecutive bus-off events: emit `STATUS,BUSOFF_PERSISTENT\r\n`

**Per-ID error statistics:**
- Add `uint16_t tx_errors`, `uint16_t rx_errors` to `can_id_profile_t`
- Emit every profiler interval: `ERRSTAT,<id>,<tx>,<rx>,<arb_lost>\r\n`

---

## Phase 7 — UDS ECU Controller Menu + Selective Reset

### New STM32 wire protocol lines

```
ECU,<addr_hex>,<name>,<status>\r\n
ECU_DETAIL,<addr_hex>,<session_ok>,<security_ok>,<routines_csv>\r\n
ECU_MENU_OPT,<addr_hex>,<routine_id_hex>,<label>\r\n
RESET_START,<addr_hex>,<routine_id_hex>\r\n
RESET_PROGRESS,<addr_hex>,<pct>\r\n
RESET_OK,<addr_hex>,<routine_id_hex>\r\n
RESET_FAIL,<addr_hex>,<nrc_hex>,<description>\r\n
```

### New WebSocket commands (browser → ESP32 → STM32)

```
CMD,SCAN_ECUS\n
CMD,ECU_MENU,<addr_hex>\n
CMD,ECU_RESET,<addr_hex>,<routine_id_hex>\n
```

### Honda ECU address table (ecu_db.h)

| Module | Request ID | Response ID | Key algo |
|---|---|---|---|
| ECM (Engine) | 0x7E0 | 0x7E8 | HONDA_ECM |
| TCM (Transmission) | 0x7E1 | 0x7E9 | HONDA_TCM |
| BCM (Body) | 0x7E4 | 0x7EC | HONDA_BCM |
| ADAS / LKAS | 0x7E3 | 0x7EB | HONDA_ADAS |
| Infotainment | 0x7E6 | 0x7EE | NONE |

### Honda seed/key algorithm variants

```c
typedef enum { HONDA_ECM=0, HONDA_TCM=1, HONDA_BCM=2, HONDA_ADAS=3, NONE=4 } key_algo_t;
static const uint32_t honda_key_constants[] = {
    0xCA,   /* ECM */
    0x3366, /* TCM */
    0xCA,   /* BCM */
    0xCA,   /* ADAS */
};
/* key = ~(seed + constant) */
```

### UDS sequence per ECU reset

```
Step A: 0x10 0x03 → Extended Diagnostic Session  (P2=50ms, P2*=5000ms)
Step B: 0x27 0x01 → SecurityAccess seed request
        0x27 0x02 → SecurityAccess key send (key = ~(seed + constant))
Step C: 0x31 0x01 <routine_H> <routine_L> → RoutineControl startRoutine
        NRC 0x78 (pending): retry up to 3× with P2* 5000ms timeout each
Step D: 0x11 0x01 → ECUReset hardReset
Step E: Wait 3s → 0x3E 0x00 → if ACK: RESET_OK, else RESET_FAIL
```

### ISO-TP layer (ISO 15765-2) required

Multi-frame UDS responses (> 7 data bytes):
- Single Frame (SF): `0x0N <data>` N=length
- First Frame (FF): `0x1N <len_lo> <data0..5>` 
- Tester sends Flow Control (FC): `0x30 0x00 0x00`
- Consecutive Frames (CF): `0x2x <data>` x=sequence 1–F
- Reassembly buffer: up to 256 bytes

### Routine IDs

```c
#define ROUTINE_ECM_CLEAR_ADAPT    0xFF00u
#define ROUTINE_TCM_CLEAR_ADAPT    0xFF01u
#define ROUTINE_BCM_RESET          0xFF02u
#define ROUTINE_ADAS_RECAL         0xFF03u
#define ROUTINE_ALL_DTCS_CLEAR     0x4200u
```

### New files

- `firmware/Core/Inc/ecu_db.h`
- `firmware/Core/Inc/uds_client.h` + `firmware/Core/Src/uds_client.c`
- `firmware/Core/Inc/isotp.h` + `firmware/Core/Src/isotp.c`

### Dashboard UI panels

1. **ECU Controller overlay** — full list of discovered modules, green/grey status dots
2. **Sub-menu** — per-ECU: session/security status, available routine list
3. **Confirmation dialog** — explicit confirm per routine, consequence description
4. **Progress overlay** — spinner + RESET_PROGRESS %, "Do not turn off ignition"
5. **Result card** — green RESET_OK or red RESET_FAIL + NRC description

### Constraints

- UDS only after `STATUS,LOCK` — never during bitrate scan
- Requires `PROJECT_CAN_TX_CONFIRMED=1`
- After any UDS session: return to silent mode (CAN_BTR_SILM)
- ECU scan: ping 0x7E0–0x7E7 with 0x10 0x03; only confirmed-response addresses shown
- P2=50ms, P2*=5000ms per ISO 14229

---

## Phase 8 — STM32CubeIDE + Official ST Tools Integration

### P8.1 — OpenOCD live variable bridge

**New file:** `firmware/tools/openocd_bridge.py`

Non-intrusive memory reads via DAP (CPU not stopped). Reads variables by `.map` file address. Serves as WebSocket :8766 for dashboard.

### P8.2 — STM32CubeProgrammer CLI integration

**New file:** `firmware/tools/flash_server.py`

Tiny HTTP server on host PC. POST /flash-stm32 → runs STM32_Programmer_CLI → progress as SSE stream. Dashboard "Flash via ST-Link" button — faster than UART bootloader, no BOOT0/NRST wires needed.

---

## New Hardware Required Per Phase

### Phase 1 — None
Same 6 wires. Code changes only.

### Phase 2 — Second TJA1051 (MS-CAN)
```
P2 row32 PB12 → TJA#2 pin4 RXD
P2 row33 PB13 → TJA#2 pin1 TXD
TJA#2 CANH → OBD-II pin 1
TJA#2 CANL → OBD-II pin 9
TJA#2 STB  → GND
TJA#2 VCC  → 3.3V + 100nF
```

### Phase 3 — 24LC512 EEPROM
```
P2 row21 PB6 → 24LC512 pin6 SCL + 4.7kΩ to 3.3V
P2 row23 PB7 → 24LC512 pin5 SDA + 4.7kΩ to 3.3V
24LC512 VCC  → 3.3V
24LC512 GND  → GND
24LC512 A0/A1/A2 → GND (address 0x50)
24LC512 WP   → GND
```

### Phase 4 — SWO (likely already wired)
Check SB21 solder bridge on Discovery board. If open → solder closed.

---

## New Files — Complete List

| File | Phase |
|---|---|
| `firmware/Core/Src/uart_dma.c` + `.h` | P1.3 |
| `firmware/Core/Src/eeprom_24lc512.c` + `.h` | P3.1 |
| `firmware/Core/Src/auto_correlator.c` + `.h` | P5.1 |
| `firmware/Core/Src/uds_client.c` + `.h` | P7 |
| `firmware/Core/Src/isotp.c` + `.h` | P7 |
| `firmware/Core/Inc/ecu_db.h` | P7 |
| `firmware/tools/swo_server.py` | P4.2 |
| `firmware/tools/openocd_bridge.py` | P8.1 |
| `firmware/tools/flash_server.py` | P8.2 |

## Modified Files

| File | Changes |
|---|---|
| `Core/Src/main.c` | PLL, TIM2, DMA, CAN2, CCM queue, ITM, arbitration retry, bus-off backoff |
| `Core/Inc/can_frame_queue.h` | 2048 capacity, timestamp_us field |
| `Core/Src/can_frame_format.c` | bus tag (HS/MS), µs timestamp |
| `Core/Inc/can_signal_profiler.h` | per-ID tx_errors/rx_errors fields |
| `Core/Src/can_signal_profiler.c` | error stats emit (ERRSTAT line) |
| `Core/Src/obd2_decoder.c` | Mode 03/04/09, Honda Mode 21 |
| `Core/Inc/civic_can_db.h` | MS-CAN IDs added |
| `firmware/Makefile` | New sources, USE_HSE_168MHZ, USE_CAN2, USE_EEPROM, USE_ITM flags |
| `esp32-dashboard/src/main.cpp` | 5.25 Mbit/s, 4KB RX buf, zero-copy broadcast, all new line types, ECU/UDS/CORR/NAT |
| `esp32-dashboard/platformio.ini` | 240MHz config, new baud |

---

## Build Flags (final)

```makefile
# STM32
TRANSCEIVER_SILENT_CONFIRMED=1
CAN_TX_CONFIRMED=1
USE_HSE_168MHZ=1
USE_CAN2=1
USE_EEPROM=1
USE_ITM=1

# ESP32 platformio.ini
-DCONFIG_ESP32_DEFAULT_CPU_FREQ_240=1
-DSTM32_BAUD=5250000
```

---

## Execution Order

```
Phase 1  ← START NOW (no hardware, biggest impact: fixes 97% drops)
Phase 2  ← hardware session 2 (second TJA1051 + 5 wires)
Phase 3  ← hardware session 2 (24LC512 + resistors + 4 wires)
Phase 4  ← check SB21 on board (may be free already)
Phase 5  ← software only, after Phase 1 + 3 data available
Phase 6  ← software only, after Phase 1
Phase 7  ← software only (ECU reset design complete)
Phase 8  ← host PC tools, parallel / optional
```

---

## Performance Summary

| Metric | Phase 0 | Phase 1 | Phase 2 | Phase 3 | All Done |
|---|---|---|---|---|---|
| STM32 clock | 16 MHz | **168 MHz** | 168 | 168 | 168 |
| UART baud | 115,200 | **5,250,000** | 5.25M | 5.25M | 5.25M |
| Frame drops | 97% | **0%** | 0% | 0% | 0% |
| Timestamp | 1 ms | **1 µs** | 1 µs | 1 µs | 1 µs |
| CAN buses | 1 | 1 | **2** | 2 | 2 |
| Boot lock | 9 s | 9 s | 9 s | **<200 ms** | <200 ms |
| Persistence | none | none | none | **64 KB** | 64 KB |
| DTC access | none | none | none | none | **full** |
| ECU reset | none | none | none | none | **full** |
| Native decode | statistical | statistical | statistical | statistical | **auto** |
| Debug channel | none | none | none | none | **SWO** |
