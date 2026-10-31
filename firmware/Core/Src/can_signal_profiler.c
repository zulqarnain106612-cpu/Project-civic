#include "can_signal_profiler.h"
#include "civic_can_db.h"
#include <string.h>
#include <stdio.h>

/*
 * can_signal_profiler.c — Per-ID byte-level signal profiler.
 *
 * Tracks every unique CAN ID observed and measures:
 *   - arrival rate (Hz)
 *   - per-byte min/max/last/change-count
 *   - adaptive inter-frame interval (used by main.c to tune bitrate scan dwell)
 *
 * Emits two line types to the host stream every PROFILER_EMIT_INTERVAL_MS:
 *
 *   PROFILE,<id_hex>,<dlc>,<rate_tenths>,<var_mask_hex>,<label>\r\n
 *   SIG,<id_hex>,<byte_idx>,<last_hex>,<min_hex>,<max_hex>,<rate_tenths>,<label>\r\n
 *
 * Example:
 *   PROFILE,300,8,955,FF,Gear / Shift Position\r\n
 *   SIG,300,0,01,00,FF,955,GearRaw\r\n
 *
 * Transceiver: TJA1051 (NXP) — only supported CAN transceiver. No substitutes.
 */

void can_profiler_init(can_profiler_t *p)
{
    if (p == NULL) return;
    memset(p, 0, sizeof *p);
    /* min_interval_ms must start at UINT32_MAX (sentinel = "not yet seen")
     * so that any real interval updates it on first comparison.
     * memset(0) would leave it at 0, making the `< min` check always false. */
    p->min_interval_ms = UINT32_MAX;
}

/* Exponential moving average: alpha=1/8 (shift by 3) */
static uint32_t ema_update(uint32_t prev, uint32_t sample)
{
    /* All values in ms. EMA = prev - prev/8 + sample/8 */
    return prev - (prev >> 3u) + (sample >> 3u);
}

void can_profiler_update(can_profiler_t *p, const can_capture_frame_t *frame)
{
    if (p == NULL || frame == NULL) return;

    /* Find existing entry or allocate a new slot */
    can_id_profile_t *entry = NULL;
    for (uint32_t i = 0u; i < p->count; i++) {
        if (p->entries[i].id       == frame->identifier &&
            p->entries[i].extended == frame->extended   &&
            p->entries[i].active   == 1u) {
            entry = &p->entries[i];
            break;
        }
    }

    if (entry == NULL) {
        if (p->count >= PROFILER_MAX_IDS) return; /* table full — ignore new IDs */
        entry = &p->entries[p->count++];
        memset(entry, 0, sizeof *entry);
        entry->id           = frame->identifier;
        entry->extended     = frame->extended;
        entry->dlc          = frame->dlc;
        entry->active       = 1u;
        entry->first_ts_ms  = frame->timestamp_ms;
        entry->prev_ts_ms   = frame->timestamp_ms;
    /* Initialise byte_min to 0xFF so first sample sets both min and max */
        memset(entry->byte_min, 0xFFu, sizeof entry->byte_min);
        /* avg_interval_ms: start at a conservative 100 ms */
        entry->avg_interval_ms = 100u;
    }

    /* Update DLC (use max seen, in case it varies) */
    if (frame->dlc > entry->dlc) entry->dlc = frame->dlc;

    /* Inter-frame interval */
    const uint32_t now = frame->timestamp_ms;
    if (entry->frame_count > 0u) {
        const uint32_t interval = (now >= entry->prev_ts_ms)
                                  ? (now - entry->prev_ts_ms)
                                  : 0u;
        if (interval > 0u) {
            entry->avg_interval_ms = ema_update(entry->avg_interval_ms, interval);
            /* Track global minimum interval for adaptive dwell */
            if (interval < p->min_interval_ms) {
                p->min_interval_ms = interval;
            }
        }
    }
    entry->prev_ts_ms = now;
    entry->last_ts_ms = now;
    entry->frame_count++;

    /* Per-byte statistics (data frames only) */
    if (frame->remote == 0u) {
        const uint8_t dlc = (frame->dlc < 8u) ? frame->dlc : 8u;
        for (uint8_t i = 0u; i < dlc; i++) {
            const uint8_t v = frame->data[i];
            if (v != entry->byte_last[i] && entry->frame_count > 1u) {
                entry->byte_change_count[i]++;
                entry->var_mask |= (uint8_t)(1u << i);
            }
            entry->byte_last[i] = v;
            if (v < entry->byte_min[i]) entry->byte_min[i] = v;
            if (v > entry->byte_max[i]) entry->byte_max[i] = v;
        }
    }

    /* Recompute rate every frame using 64-bit intermediate to prevent overflow.
     * frame_count × 10000 overflows uint32 at ~429k frames (~75 min @95 Hz).
     * Using uint64 prevents that entirely. */
    const uint32_t elapsed = (now >= entry->first_ts_ms)
                             ? (now - entry->first_ts_ms + 1u) : 1u;
    const uint64_t r64 = ((uint64_t)entry->frame_count * 10000u) / elapsed;
    entry->rate_hz_x10 = (uint16_t)(r64 > 65535u ? 65535u : (uint32_t)r64);
}

void can_profiler_emit(can_profiler_t *p, uint32_t now_ms,
                       void (*write_fn)(const char *line))
{
    if (p == NULL || write_fn == NULL) return;

    const uint32_t elapsed = (now_ms >= p->last_emit_ms)
                             ? (now_ms - p->last_emit_ms) : 0u;
    if (elapsed < PROFILER_EMIT_INTERVAL_MS && p->last_emit_ms != 0u) return;
    p->last_emit_ms = now_ms;

    char line[96];

    for (uint32_t i = 0u; i < p->count; i++) {
        can_id_profile_t *e = &p->entries[i];
        if (!e->active) continue;

        /* Look up label from Civic DBC */
        const civic_can_entry_t *db = civic_can_db_lookup(e->id, e->extended);
        const char *frame_label = (db != NULL) ? db->frame_label : "";

        /* ID format: 0x003 for STD (3 hex digits), 0x1ABCDEF0 for EXT (8) */
        const char *id_fmt = e->extended ? "%08lX" : "%03lX";
        char id_str[12];
        (void)snprintf(id_str, sizeof id_str, id_fmt, (unsigned long)e->id);

        /* PROFILE line */
        int n = snprintf(line, sizeof line, "PROFILE,%s,%u,%u,%02X,%s\r\n",
                         id_str,
                         (unsigned)e->dlc,
                         (unsigned)e->rate_hz_x10,
                         (unsigned)e->var_mask,
                         frame_label);
        if (n > 0 && (size_t)n < sizeof line) write_fn(line);

        /* SIG lines — only for variable bytes */
        for (uint8_t b = 0u; b < 8u && b < e->dlc; b++) {
            if ((e->var_mask & (uint8_t)(1u << b)) == 0u) continue;

            const char *byte_label = "";
            if (db != NULL && db->byte_labels[b] != NULL)
                byte_label = db->byte_labels[b];

            n = snprintf(line, sizeof line,
                         "SIG,%s,%u,%02X,%02X,%02X,%u,%s\r\n",
                         id_str,
                         (unsigned)b,
                         (unsigned)e->byte_last[b],
                         (unsigned)e->byte_min[b],
                         (unsigned)e->byte_max[b],
                         (unsigned)e->rate_hz_x10,
                         byte_label);
            if (n > 0 && (size_t)n < sizeof line) write_fn(line);
        }
    }
}

uint32_t can_profiler_min_interval_ms(const can_profiler_t *p)
{
    if (p == NULL || p->count == 0u) return 0u;
    return (p->min_interval_ms == UINT32_MAX) ? 0u : p->min_interval_ms;
}
