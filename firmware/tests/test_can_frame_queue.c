#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../Core/Inc/can_frame_queue.h"
#include "../Core/Inc/can_frame_format.h"

int main(void)
{
    can_capture_queue_t q;
    can_capture_frame_t in = {0}, out = {0};
    char csv[96];

    can_capture_queue_init(&q);
    assert(!can_capture_queue_pop(&q, &out));
    assert(!can_capture_queue_pop(0, &out));
    assert(!can_capture_queue_pop(&q, 0));
    can_capture_queue_init(0);

    in.timestamp_ms = 1234u;
    in.identifier = 0x18DAF110u;
    in.extended = 1u;
    in.dlc = 8u;
    in.data[0] = 2u;
    in.data[7] = 0xA5u;
    assert(can_capture_queue_push_isr(&q, &in));
    assert(can_capture_queue_pop(&q, &out));
    assert(out.timestamp_ms == in.timestamp_ms);
    assert(out.identifier == in.identifier);
    assert(out.extended == 1u && out.dlc == 8u);
    assert(out.data[0] == 2u && out.data[7] == 0xA5u);
    assert(!can_capture_queue_pop(&q, &out));

    /* Ring capacity, wraparound, and deterministic drop accounting. */
    can_capture_queue_init(&q);
    for (unsigned i = 0; i < CAN_CAPTURE_QUEUE_CAPACITY - 1u; i++) {
        in.identifier = i;
        in.extended = 0u;
        assert(can_capture_queue_push_isr(&q, &in));
    }
    assert(!can_capture_queue_push_isr(&q, &in));
    assert(q.dropped == 1u);
    for (unsigned i = 0; i < CAN_CAPTURE_QUEUE_CAPACITY - 1u; i++) {
        assert(can_capture_queue_pop(&q, &out));
        assert(out.identifier == i);
    }
    assert(!can_capture_queue_pop(&q, &out));
    for (unsigned i = 0; i < 300u; i++) {
        in.identifier = i & 0x7ffu;
        assert(can_capture_queue_push_isr(&q, &in));
        assert(can_capture_queue_pop(&q, &out));
        assert(out.identifier == (i & 0x7ffu));
    }

    in.timestamp_ms = 1234u;
    in.timestamp_us = 1234u;
    in.identifier = 0x18DAF110u;
    in.extended = 1u;
    in.remote = 0u;
    in.dlc = 2u;
    in.data[0] = 0x02u;
    in.data[1] = 0xA5u;
    assert(can_capture_frame_to_csv(&in, csv, sizeof csv) > 0u);
    assert(strcmp(csv, "1234,EXT,18DAF110,2,0,02A5\n") == 0);
    assert(can_capture_frame_to_csv(&in, csv, 4u) == 0u);
    assert(can_capture_frame_to_csv(0, csv, sizeof csv) == 0u);
    assert(can_capture_frame_to_csv(&in, 0, sizeof csv) == 0u);
    assert(can_capture_frame_to_csv(&in, csv, 0u) == 0u);

    in.extended = 0u;
    in.identifier = 0x7DFu;
    in.remote = 1u;
    in.dlc = 0u;
    assert(can_capture_frame_to_csv(&in, csv, sizeof csv) > 0u);
    assert(strcmp(csv, "1234,STD,7DF,0,1,\n") == 0);

    /* RTR DLC is requested length, not a payload byte count. */
    in.dlc = 8u;
    memset(in.data, 0xEE, sizeof in.data);
    assert(can_capture_frame_to_csv(&in, csv, sizeof csv) > 0u);
    assert(strcmp(csv, "1234,STD,7DF,8,1,\n") == 0);

    /* Reject invalid DLC and IDs rather than silently formatting malformed frames. */
    can_capture_queue_init(&q);
    in.remote = 0u;
    in.dlc = 9u;
    assert(can_capture_frame_to_csv(&in, csv, sizeof csv) == 0u);
    in.dlc = 1u;
    in.identifier = 0x800u;
    assert(can_capture_frame_to_csv(&in, csv, sizeof csv) == 0u);
    assert(!can_capture_queue_push_isr(&q, &in));
    assert(q.dropped == 1u);
    in.identifier = 0x7ffu;
    in.dlc = 9u;
    assert(!can_capture_queue_push_isr(&q, &in));
    assert(q.dropped == 2u);
    in.dlc = 0u;
    in.extended = 1u;
    in.identifier = 0x20000000u;
    assert(can_capture_frame_to_csv(&in, csv, sizeof csv) == 0u);
    assert(!can_capture_queue_push_isr(&q, &in));
    assert(q.dropped == 3u);
    assert(!can_capture_queue_push_isr(0, &in));
    assert(!can_capture_queue_push_isr(&q, 0));

    puts("PASS: queue wraparound, metadata/payload, validation, overflow/drop accounting, and CSV/RTR formatting");
    return 0;
}
