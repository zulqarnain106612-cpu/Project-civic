#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../Core/Inc/can_signal_profiler.h"
#include "../Core/Inc/can_frame_queue.h"

static int emit_count = 0;
static char last_line[128];
static void test_write(const char *line)
{
    emit_count++;
    strncpy(last_line, line, sizeof last_line - 1);
    last_line[sizeof last_line - 1] = '\0';
}

static can_capture_frame_t make_frame(uint32_t id, uint32_t ts, const uint8_t *data, uint8_t dlc)
{
    can_capture_frame_t f = {0};
    f.identifier   = id;
    f.extended     = 0u;
    f.remote       = 0u;
    f.dlc          = dlc;
    f.timestamp_ms = ts;
    if (data) {
        for (uint8_t i = 0u; i < dlc && i < 8u; i++) f.data[i] = data[i];
    }
    return f;
}

int main(void)
{
    can_profiler_t p;

    /* ── Init ─────────────────────────────────────────────────────── */
    can_profiler_init(&p);
    assert(p.count == 0u);
    assert(can_profiler_min_interval_ms(&p) == 0u);

    /* ── Single frame: min/max should equal the byte value ─────────── */
    {
        uint8_t d0[] = {0xAB, 0x00, 0xFF, 0x55, 0x00, 0x00, 0x00, 0x00};
        can_capture_frame_t f = make_frame(0x300, 100u, d0, 8u);
        can_profiler_update(&p, &f);
        assert(p.count == 1u);
        can_id_profile_t *e = &p.entries[0];
        assert(e->id == 0x300u);
        assert(e->frame_count == 1u);
        assert(e->byte_min[0] == 0xABu);
        assert(e->byte_max[0] == 0xABu);
        assert(e->byte_last[0] == 0xABu);
        assert(e->var_mask == 0u); /* no change yet on first frame */
    }

    /* ── Second frame with different byte 0: var_mask bit 0 set ────── */
    {
        uint8_t d1[] = {0x31, 0x00, 0xFF, 0x55, 0x00, 0x00, 0x00, 0x00};
        can_capture_frame_t f = make_frame(0x300, 110u, d1, 8u);
        can_profiler_update(&p, &f);
        can_id_profile_t *e = &p.entries[0];
        assert(e->frame_count == 2u);
        assert(e->byte_min[0] == 0x31u);
        assert(e->byte_max[0] == 0xABu);
        assert(e->byte_last[0] == 0x31u);
        assert(e->var_mask & 0x01u); /* bit 0 = byte 0 changed */
        /* byte 1 constant (0x00) — should NOT be in var_mask */
        assert(!(e->var_mask & 0x02u));
    }

    /* ── New ID creates a second entry ─────────────────────────────── */
    {
        uint8_t d[] = {0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        can_capture_frame_t f = make_frame(0x1A0, 200u, d, 8u);
        can_profiler_update(&p, &f);
        assert(p.count == 2u);
        assert(p.entries[1].id == 0x1A0u);
    }

    /* ── Inter-frame interval tracking ─────────────────────────────── */
    {
        /* Send 3 more frames for 0x300 at 10 ms intervals */
        uint8_t d[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        for (uint32_t t = 120u; t <= 140u; t += 10u) {
            can_capture_frame_t f = make_frame(0x300, t, d, 8u);
            can_profiler_update(&p, &f);
        }
        /* min_interval_ms should be ≤ 10 */
        assert(can_profiler_min_interval_ms(&p) <= 10u);
    }

    /* ── Emit: no output before interval elapsed ────────────────────── */
    {
        emit_count = 0;
        can_profiler_emit(&p, 100u, test_write); /* first call sets last_emit_ms */
        /* second call immediately: 0 ms elapsed — should emit because last_emit_ms was 0 */
        emit_count = 0;
        can_profiler_emit(&p, 150u, test_write); /* 50 ms elapsed, < 2000 — no emit */
        assert(emit_count == 0);
    }

    /* ── Emit: output after PROFILER_EMIT_INTERVAL_MS elapsed ─────── */
    {
        emit_count = 0;
        can_profiler_emit(&p, 100u + PROFILER_EMIT_INTERVAL_MS + 1u, test_write);
        assert(emit_count > 0); /* must have emitted at least one PROFILE line */
        /* last_line should contain a PROFILE, or SIG */
        assert(strncmp(last_line, "PROFILE,", 8) == 0 ||
               strncmp(last_line, "SIG,", 4) == 0);
    }

    /* ── Table full: 129th ID is silently dropped ─────────────────── */
    {
        can_profiler_init(&p);
        uint8_t d[] = {0x01};
        for (uint32_t id = 0u; id < PROFILER_MAX_IDS; id++) {
            can_capture_frame_t f = make_frame(id, id * 10u, d, 1u);
            can_profiler_update(&p, &f);
        }
        assert(p.count == PROFILER_MAX_IDS);
        /* 129th ID — should not crash and count stays at max */
        {
            can_capture_frame_t f = make_frame(0xFFFFu, 9999u, d, 1u);
            can_profiler_update(&p, &f);
            assert(p.count == PROFILER_MAX_IDS);
        }
    }

    /* ── NULL safety ──────────────────────────────────────────────── */
    can_profiler_init(NULL); /* no crash */
    can_profiler_update(NULL, NULL); /* no crash */
    can_profiler_emit(NULL, 0u, test_write); /* no crash */
    {
        can_profiler_t p2;
        can_profiler_init(&p2);
        can_profiler_update(&p2, NULL); /* no crash */
    }

    puts("PASS: can_signal_profiler — init, update, min/max, var_mask, rate, emit, table-full, nulls");
    return 0;
}
