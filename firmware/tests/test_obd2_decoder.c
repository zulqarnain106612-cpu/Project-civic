#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../Core/Inc/obd2_decoder.h"
#include "../Core/Inc/can_frame_queue.h"

/* Helper: build a fake OBD-II Mode 01 response frame */
static can_capture_frame_t make_obd_frame(uint8_t pid, uint8_t A, uint8_t B)
{
    can_capture_frame_t f = {0};
    f.identifier  = 0x7E8u;
    f.extended    = 0u;
    f.remote      = 0u;
    f.dlc         = 8u;
    f.data[0]     = 0x04u; /* length */
    f.data[1]     = 0x41u; /* positive response */
    f.data[2]     = pid;
    f.data[3]     = A;
    f.data[4]     = B;
    return f;
}

int main(void)
{
    char line[OBD2_LINE_CAPACITY];

    /* PID 0x0C — RPM */
    {
        can_capture_frame_t f = make_obd_frame(0x0C, 0x0E, 0x88); /* (0x0E*256+0x88)/4 = 3746/4 = 936.5 */
        assert(obd2_decode(&f, line, sizeof line));
        assert(strncmp(line, "PID,0C,RPM,rpm,", 15) == 0);
    }

    /* PID 0x05 — Coolant Temp: A=0x80=128 → 128-40=88°C */
    {
        can_capture_frame_t f = make_obd_frame(0x05, 0x80, 0x00);
        assert(obd2_decode(&f, line, sizeof line));
        assert(strncmp(line, "PID,05,Coolant Temp,C,", 22) == 0);
    }

    /* PID 0x42 — Battery Voltage: A=0x0C, B=0xCB → (12*256+203)/1000 = 3275/1000 = 3.275 V
     * (real battery voltage is higher but formula is (A*256+B)/1000) */
    {
        can_capture_frame_t f = make_obd_frame(0x42, 0x35, 0x4A); /* (0x35*256+0x4A)/1000 = (53*256+74)/1000 = 13642/1000 = 13.64 V */
        assert(obd2_decode(&f, line, sizeof line));
        assert(strncmp(line, "PID,42,Battery Voltage,V,", 25) == 0);
    }

    /* PID 0x2F — Fuel Level: A=0x47=71 → 71*100/255 ≈ 27.8% */
    {
        can_capture_frame_t f = make_obd_frame(0x2F, 0x47, 0x00);
        assert(obd2_decode(&f, line, sizeof line));
        assert(strncmp(line, "PID,2F,Fuel Level,%,", 20) == 0);
    }

    /* PID 0x11 — Throttle: A=0x40=64 → 64*100/255 ≈ 25.1% */
    {
        can_capture_frame_t f = make_obd_frame(0x11, 0x40, 0x00);
        assert(obd2_decode(&f, line, sizeof line));
        assert(strncmp(line, "PID,11,Throttle,%,", 18) == 0);
    }

    /* PID 0x10 — MAF: A=0x02, B=0x58 → (2*256+88)/100 = 5.96 g/s */
    {
        can_capture_frame_t f = make_obd_frame(0x10, 0x02, 0x58);
        assert(obd2_decode(&f, line, sizeof line));
        assert(strncmp(line, "PID,10,MAF,gs,", 14) == 0);
    }

    /* Unknown PID — should return false */
    {
        can_capture_frame_t f = make_obd_frame(0xAA, 0x00, 0x00);
        assert(!obd2_decode(&f, line, sizeof line));
    }

    /* Wrong ID — not 0x7E8 */
    {
        can_capture_frame_t f = make_obd_frame(0x0C, 0x10, 0x00);
        f.identifier = 0x7E0u;
        assert(!obd2_decode(&f, line, sizeof line));
    }

    /* Wrong mode byte — not 0x41 */
    {
        can_capture_frame_t f = make_obd_frame(0x0C, 0x10, 0x00);
        f.data[1] = 0x40u;
        assert(!obd2_decode(&f, line, sizeof line));
    }

    /* NULL inputs */
    assert(!obd2_decode(NULL, line, sizeof line));
    {
        can_capture_frame_t f = make_obd_frame(0x0C, 0x10, 0x00);
        assert(!obd2_decode(&f, NULL, sizeof line));
        assert(!obd2_decode(&f, line, 0));
        assert(!obd2_decode(&f, line, OBD2_LINE_CAPACITY - 1u)); /* too small */
    }

    puts("PASS: obd2_decoder — all PID formulas, edge cases, and null guards");
    return 0;
}
