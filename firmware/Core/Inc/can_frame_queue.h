#ifndef CAN_FRAME_QUEUE_H
#define CAN_FRAME_QUEUE_H
#include <stdbool.h>
#include <stdint.h>
#define CAN_CAPTURE_QUEUE_CAPACITY 2048u
#define CAN_CAPTURE_MAX_DLC 8u
typedef struct { uint32_t timestamp_ms; uint32_t timestamp_us; uint32_t identifier; uint8_t extended; uint8_t remote; uint8_t dlc; uint8_t data[CAN_CAPTURE_MAX_DLC]; } can_capture_frame_t;
typedef struct { can_capture_frame_t frames[CAN_CAPTURE_QUEUE_CAPACITY]; volatile uint16_t head; volatile uint16_t tail; volatile uint32_t dropped; } can_capture_queue_t;
void can_capture_queue_init(can_capture_queue_t *q);
bool can_capture_queue_push_isr(can_capture_queue_t *q, const can_capture_frame_t *frame);
bool can_capture_queue_pop(can_capture_queue_t *q, can_capture_frame_t *out);

/* Runtime handoff API for a future verified transport adapter. */
bool project_capture_pop(can_capture_frame_t *out);
uint32_t project_capture_dropped(void);
uint32_t project_capture_errors(void);
uint32_t project_capture_millis(void);
#endif
