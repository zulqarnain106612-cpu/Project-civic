#ifndef CAN_FRAME_FORMAT_H
#define CAN_FRAME_FORMAT_H
#include <stddef.h>
#include "can_frame_queue.h"
/* CSV record: timestamp_ms,format,id,dlc,rtr,payload_hex\n */
size_t can_capture_frame_to_csv(const can_capture_frame_t *frame, char *out, size_t capacity);
#endif
