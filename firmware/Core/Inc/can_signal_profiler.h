#ifndef CAN_SIGNAL_PROFILER_H
#define CAN_SIGNAL_PROFILER_H
/*
 * can_signal_profiler.h — Per-ID byte-level statistical profiler.
 *
 * Tracks for each unique (id, extended) pair:
 *   - frame count and arrival rate
 *   - per-byte min/max/last value and change count
 *
 * Outputs "PROFILE,...\r\n" and "SIG,...\r\n" lines for the host stream.
 *
 * All functions called from the main loop (not ISR-safe for profiler state).
 *
 * Transceiver: TJA1051 (NXP) — only supported CAN transceiver. No substitutes.
 */

#include "can_frame_queue.h"
#include <stddef.h>
#include <stdbool.h>

/* Maximum unique CAN IDs tracked. After this, new IDs are silently ignored. */
#define PROFILER_MAX_IDS 128u

/*
 * Per-ID statistics entry.
 */
typedef struct {
    uint32_t id;
    uint8_t  extended;
    uint8_t  dlc;
    uint8_t  active;            /* 1 = slot in use */

    uint32_t frame_count;
    uint32_t first_ts_ms;
    uint32_t last_ts_ms;
    uint32_t prev_ts_ms;        /* for inter-frame interval tracking */

    /* Adaptive dwell: rolling average inter-frame interval (ms) */
    uint32_t avg_interval_ms;   /* exponential moving average, Q8 fixed-point */

    uint16_t rate_hz_x10;       /* rate in 0.1 Hz units */
    uint8_t  var_mask;          /* bitmask of bytes that changed at least once */

    uint8_t  byte_min[8];
    uint8_t  byte_max[8];
    uint8_t  byte_last[8];
    uint32_t byte_change_count[8];
} can_id_profile_t;

/*
 * Global profiler state — zero-initialised at startup by can_profiler_init().
 */
typedef struct {
    can_id_profile_t entries[PROFILER_MAX_IDS];
    uint32_t         count;          /* number of active entries */
    uint32_t         last_emit_ms;   /* last time PROFILE/SIG lines were emitted */

    /* Adaptive bitrate dwell: fastest observed inter-frame interval across
     * all IDs, used to tune scan dwell time in main.c */
    uint32_t         min_interval_ms;
} can_profiler_t;

/* Emit interval: send PROFILE + SIG lines every this many ms */
#define PROFILER_EMIT_INTERVAL_MS 2000u

void can_profiler_init(can_profiler_t *p);

/*
 * can_profiler_update() — feed one dequeued frame into the profiler.
 * Not ISR-safe; call from main loop only.
 */
void can_profiler_update(can_profiler_t *p, const can_capture_frame_t *frame);

/*
 * can_profiler_emit() — if PROFILER_EMIT_INTERVAL_MS has elapsed since last
 * emit, write PROFILE and SIG lines via write_fn callback.
 * write_fn is called with each complete \r\n-terminated line.
 */
void can_profiler_emit(can_profiler_t *p, uint32_t now_ms,
                       void (*write_fn)(const char *line));

/*
 * can_profiler_min_interval_ms() — returns the fastest observed inter-frame
 * interval across all IDs, used to compute adaptive scan dwell.
 * Returns 0 if no frames seen yet.
 */
uint32_t can_profiler_min_interval_ms(const can_profiler_t *p);

#endif /* CAN_SIGNAL_PROFILER_H */
