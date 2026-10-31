#ifndef OBD2_DECODER_H
#define OBD2_DECODER_H
/*
 * obd2_decoder.h — ISO 15765-4 OBD-II Mode 01 single-frame response decoder.
 *
 * Decodes positive responses (byte[1]==0x41) from ECM ID 0x7E8.
 * Outputs "PID,<hex>,<label>,<unit>,<value>\r\n" into caller-supplied buffer.
 *
 * Transceiver: TJA1051 (NXP) — only supported CAN transceiver. No substitutes.
 */

#include "can_frame_queue.h"
#include <stddef.h>
#include <stdbool.h>

/*
 * obd2_decode() — attempt to decode frame as an OBD-II Mode 01 response.
 *
 * Returns true and fills out_line if the frame is a valid OBD-II response
 * with a known PID. Returns false (out_line unchanged) otherwise.
 *
 * out_line must be at least OBD2_LINE_CAPACITY bytes.
 */
#define OBD2_LINE_CAPACITY 64u

bool obd2_decode(const can_capture_frame_t *frame, char *out_line, size_t capacity);

#endif /* OBD2_DECODER_H */
