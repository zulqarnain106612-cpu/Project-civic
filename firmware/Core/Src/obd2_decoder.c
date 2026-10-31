#include "obd2_decoder.h"
#include <stdio.h>
#include <string.h>

/*
 * OBD-II Mode 01 positive response decoder (ISO 15765-4).
 *
 * Single-frame layout:
 *   data[0] = ISO-TP PCI length byte  (not validated here)
 *   data[1] = 0x41                    (Mode 01 positive response)
 *   data[2] = PID
 *   data[3] = A byte
 *   data[4] = B byte  (when DLC >= 5)
 *   data[5] = C byte  (when DLC >= 6)
 *   data[6] = D byte  (when DLC >= 7)
 *
 * Only STD frames from ECM (0x7E8) are accepted.
 *
 * Output: "PID,<HEX>,<label>,<unit>,<value>\r\n"
 * IDs are emitted without "0x" prefix to match the CSV and profiler format.
 *
 * Sensors covered:
 *   Engine: RPM, load, coolant, intake manifold pressure, MAF, MAF-B,
 *           intake air temp, ambient air temp, oil temp, engine runtime,
 *           throttle (absolute + relative + commanded), ignition timing advance
 *   Fuel:   fuel system status, short/long-term trim bank 1/2,
 *           fuel pressure (gauge+rail+rail-absolute), fuel level,
 *           evap purge, evap system vapour pressure, warm-ups, distance
 *   O2:     O2 sensors present, O2 voltage+trim (banks 1-2 sensors 1-4),
 *           wide-band lambda+voltage (PID 0x24-0x2B)
 *   Exhaust/emissions: EGR command + error, catalyst temp banks 1-2 sensors 1-2,
 *                      secondary air status, barometric pressure
 *   Speed:  vehicle speed
 *   Electrical: control module voltage (battery voltage)
 *   Misc:  absolute load, distance with MIL on, time with MIL on,
 *           time since codes cleared, OBD standard, commanded equiv ratio
 */

bool obd2_decode(const can_capture_frame_t *frame, char *out_line, size_t capacity)
{
    if (!frame || !out_line || capacity < OBD2_LINE_CAPACITY) return false;
    if (frame->extended || frame->remote) return false;
    if (frame->identifier != 0x7E8u) return false;
    if (frame->dlc < 4u) return false;
    if (frame->data[1] != 0x41u) return false;

    const uint8_t pid = frame->data[2];
    const uint8_t A   = frame->data[3];
    const uint8_t B   = (frame->dlc >= 5u) ? frame->data[4] : 0u;
    const uint8_t C   = (frame->dlc >= 6u) ? frame->data[5] : 0u;
    const uint8_t D   = (frame->dlc >= 7u) ? frame->data[6] : 0u;
    (void)D; /* suppress unused-variable warning for PIDs that don't use D */

    const char *label = NULL;
    const char *unit  = "";
    char        vs[24];  /* value string */
    float       val   = 0.0f;

    switch (pid) {

    /* ── PID 0x03 Fuel system status ─────────────────────────────── */
    case 0x03u:
        label = "Fuel System Status";
        unit  = "";
        /* A = sys1 status bitmask, not a scalar; emit as hex */
        (void)snprintf(vs, sizeof vs, "%02X%02X", (unsigned)A, (unsigned)B);
        goto emit_raw;

    /* ── PID 0x04 Calculated engine load ──────────────────────────── */
    case 0x04u:
        label = "Engine Load"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x05 Coolant temperature ─────────────────────────────── */
    case 0x05u:
        label = "Coolant Temp"; unit = "C";
        val = (float)A - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x06 Short-term fuel trim bank 1 ─────────────────────── */
    case 0x06u:
        label = "Fuel Trim ST B1"; unit = "%";
        val = ((float)A - 128.0f) * (100.0f / 128.0f);
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;

    /* ── PID 0x07 Long-term fuel trim bank 1 ──────────────────────── */
    case 0x07u:
        label = "Fuel Trim LT B1"; unit = "%";
        val = ((float)A - 128.0f) * (100.0f / 128.0f);
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;

    /* ── PID 0x08 Short-term fuel trim bank 2 ─────────────────────── */
    case 0x08u:
        label = "Fuel Trim ST B2"; unit = "%";
        val = ((float)A - 128.0f) * (100.0f / 128.0f);
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;

    /* ── PID 0x09 Long-term fuel trim bank 2 ──────────────────────── */
    case 0x09u:
        label = "Fuel Trim LT B2"; unit = "%";
        val = ((float)A - 128.0f) * (100.0f / 128.0f);
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;

    /* ── PID 0x0A Fuel pressure (gauge) ───────────────────────────── */
    case 0x0Au:
        label = "Fuel Pressure"; unit = "kPa";
        val = (float)A * 3.0f;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x0B Intake manifold pressure ────────────────────────── */
    case 0x0Bu:
        label = "Intake Manifold Pressure"; unit = "kPa";
        val = (float)A;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x0C RPM ─────────────────────────────────────────────── */
    case 0x0Cu:
        label = "RPM"; unit = "rpm";
        val = ((float)A * 256.0f + (float)B) / 4.0f;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x0D Vehicle speed ────────────────────────────────────── */
    case 0x0Du:
        label = "Vehicle Speed"; unit = "kmh";
        val = (float)A;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x0E Ignition timing advance ──────────────────────────── */
    case 0x0Eu:
        label = "Timing Advance"; unit = "deg";
        val = (float)A / 2.0f - 64.0f;
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x0F Intake air temperature ───────────────────────────── */
    case 0x0Fu:
        label = "Intake Air Temp"; unit = "C";
        val = (float)A - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x10 MAF air flow rate ────────────────────────────────── */
    case 0x10u:
        label = "MAF"; unit = "gs";
        val = ((float)A * 256.0f + (float)B) / 100.0f;
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;

    /* ── PID 0x11 Throttle position ────────────────────────────────── */
    case 0x11u:
        label = "Throttle"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x12 Commanded secondary air status ──────────────────── */
    case 0x12u:
        label = "Secondary Air"; unit = "";
        (void)snprintf(vs, sizeof vs, "%02X", (unsigned)A);
        goto emit_raw;

    /* ── PID 0x13 O2 sensors present (bitmask) ─────────────────────── */
    case 0x13u:
        label = "O2 Sensors Present"; unit = "";
        (void)snprintf(vs, sizeof vs, "%02X", (unsigned)A);
        goto emit_raw;

    /* ── PIDs 0x14-0x1B  O2 sensor voltage + short-term fuel trim ──── */
    case 0x14u: case 0x15u: case 0x16u: case 0x17u:
    case 0x18u: case 0x19u: case 0x1Au: case 0x1Bu: {
        static const char *const o2lbl[] = {
            "O2 B1S1","O2 B1S2","O2 B1S3","O2 B1S4",
            "O2 B2S1","O2 B2S2","O2 B2S3","O2 B2S4"
        };
        label = o2lbl[pid - 0x14u]; unit = "V";
        val = (float)A / 200.0f;  /* A = voltage × 200, 0-1.275 V */
        (void)snprintf(vs, sizeof vs, "%.3f", (double)val); break;
    }

    /* ── PID 0x1C OBD standard ─────────────────────────────────────── */
    case 0x1Cu:
        label = "OBD Standard"; unit = "";
        (void)snprintf(vs, sizeof vs, "%u", (unsigned)A);
        goto emit_raw;

    /* ── PID 0x1F Engine run time ──────────────────────────────────── */
    case 0x1Fu:
        label = "Engine Runtime"; unit = "s";
        val = (float)A * 256.0f + (float)B;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x21 Distance with MIL on ─────────────────────────────── */
    case 0x21u:
        label = "Distance MIL On"; unit = "km";
        val = (float)A * 256.0f + (float)B;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x22 Fuel rail pressure (relative to manifold vacuum) ─── */
    case 0x22u:
        label = "Fuel Rail Pressure"; unit = "kPa";
        val = 0.079f * ((float)A * 256.0f + (float)B);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x23 Fuel rail gauge pressure ─────────────────────────── */
    case 0x23u:
        label = "Fuel Rail Gauge Pressure"; unit = "kPa";
        val = 10.0f * ((float)A * 256.0f + (float)B);
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PIDs 0x24-0x2B  Wide-band O2: equiv ratio + voltage ──────── */
    case 0x24u: case 0x25u: case 0x26u: case 0x27u:
    case 0x28u: case 0x29u: case 0x2Au: case 0x2Bu: {
        static const char *const wblbl[] = {
            "WBO2 B1S1","WBO2 B1S2","WBO2 B1S3","WBO2 B1S4",
            "WBO2 B2S1","WBO2 B2S2","WBO2 B2S3","WBO2 B2S4"
        };
        label = wblbl[pid - 0x24u]; unit = "lambda";
        /* A,B = equivalence ratio ×32768 */
        val = ((float)A * 256.0f + (float)B) / 32768.0f;
        (void)snprintf(vs, sizeof vs, "%.4f", (double)val); break;
    }

    /* ── PID 0x2C EGR commanded ─────────────────────────────────────── */
    case 0x2Cu:
        label = "EGR Commanded"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x2D EGR error ─────────────────────────────────────────── */
    case 0x2Du:
        label = "EGR Error"; unit = "%";
        val = ((float)A - 128.0f) * (100.0f / 128.0f);
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;

    /* ── PID 0x2E Commanded evap purge ─────────────────────────────── */
    case 0x2Eu:
        label = "Evap Purge"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x2F Fuel level ────────────────────────────────────────── */
    case 0x2Fu:
        label = "Fuel Level"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x30 Warm-ups since codes cleared ──────────────────────── */
    case 0x30u:
        label = "Warm-ups Since Clear"; unit = "";
        val = (float)A;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x31 Distance since codes cleared ──────────────────────── */
    case 0x31u:
        label = "Distance Since Clear"; unit = "km";
        val = (float)A * 256.0f + (float)B;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x32 Evap system vapour pressure ──────────────────────── */
    case 0x32u:
        label = "Evap Vapour Pressure"; unit = "Pa";
        /* signed 16-bit, scale 0.25 Pa */
        val = (float)((int16_t)((uint16_t)A * 256u + B)) * 0.25f;
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;

    /* ── PID 0x33 Absolute barometric pressure ──────────────────────── */
    case 0x33u:
        label = "Barometric Pressure"; unit = "kPa";
        val = (float)A;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PIDs 0x3C-0x3F  Catalyst temperature ──────────────────────── */
    case 0x3Cu:
        label = "Catalyst Temp B1S1"; unit = "C";
        val = ((float)A * 256.0f + (float)B) / 10.0f - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;
    case 0x3Du:
        label = "Catalyst Temp B2S1"; unit = "C";
        val = ((float)A * 256.0f + (float)B) / 10.0f - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;
    case 0x3Eu:
        label = "Catalyst Temp B1S2"; unit = "C";
        val = ((float)A * 256.0f + (float)B) / 10.0f - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;
    case 0x3Fu:
        label = "Catalyst Temp B2S2"; unit = "C";
        val = ((float)A * 256.0f + (float)B) / 10.0f - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x42 Control module voltage (battery) ──────────────────── */
    case 0x42u:
        label = "Battery Voltage"; unit = "V";
        val = ((float)A * 256.0f + (float)B) / 1000.0f;
        (void)snprintf(vs, sizeof vs, "%.3f", (double)val); break;

    /* ── PID 0x43 Absolute engine load ─────────────────────────────── */
    case 0x43u:
        label = "Abs Engine Load"; unit = "%";
        /* SAE J1979: ((A×256)+B) × 100 / 255  (max ~25700%) */
        val = ((float)A * 256.0f + (float)B) * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x44 Commanded equivalence ratio ──────────────────────── */
    case 0x44u:
        label = "Commanded EQ Ratio"; unit = "lambda";
        val = ((float)A * 256.0f + (float)B) / 32768.0f;
        (void)snprintf(vs, sizeof vs, "%.4f", (double)val); break;

    /* ── PID 0x45 Relative throttle position ───────────────────────── */
    case 0x45u:
        label = "Relative Throttle"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x46 Ambient air temperature ──────────────────────────── */
    case 0x46u:
        label = "Ambient Air Temp"; unit = "C";
        val = (float)A - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x47 Absolute throttle position B ─────────────────────── */
    case 0x47u:
        label = "Throttle Pos B"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x49 Accelerator pedal position D ──────────────────────── */
    case 0x49u:
        label = "Accel Pedal D"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x4A Accelerator pedal position E ──────────────────────── */
    case 0x4Au:
        label = "Accel Pedal E"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x4C Commanded throttle actuator ──────────────────────── */
    case 0x4Cu:
        label = "Commanded Throttle"; unit = "%";
        val = A * (100.0f / 255.0f);
        (void)snprintf(vs, sizeof vs, "%.1f", (double)val); break;

    /* ── PID 0x4D Time with MIL on ──────────────────────────────────── */
    case 0x4Du:
        label = "Time MIL On"; unit = "min";
        val = (float)A * 256.0f + (float)B;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x4E Time since codes cleared ─────────────────────────── */
    case 0x4Eu:
        label = "Time Since Clear"; unit = "min";
        val = (float)A * 256.0f + (float)B;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x5C Engine oil temperature ───────────────────────────── */
    case 0x5Cu:
        label = "Oil Temp"; unit = "C";
        val = (float)A - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    /* ── PID 0x5E Engine fuel rate ──────────────────────────────────── */
    case 0x5Eu:
        label = "Fuel Rate"; unit = "Lh";
        val = ((float)A * 256.0f + (float)B) * 0.05f;
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;

    /* ── PIDs 0x66/0x67  MAF sensor B ──────────────────────────────── */
    case 0x66u:
        label = "MAF B"; unit = "gs";
        val = ((float)B * 256.0f + (float)C) / 32.0f;
        (void)snprintf(vs, sizeof vs, "%.2f", (double)val); break;
    case 0x67u:
        label = "Engine Coolant Temp 2"; unit = "C";
        val = (float)B - 40.0f;
        (void)snprintf(vs, sizeof vs, "%.0f", (double)val); break;

    default:
        return false; /* PID not decoded — not a bug */
    }

    /* Format: "PID,<HEX>,<label>,<unit>,<value>\r\n"
     * IDs are plain hex, no "0x" prefix, matching CSV and profiler format. */
    {
        int n = snprintf(out_line, capacity, "PID,%02X,%s,%s,%s\r\n",
                         (unsigned)pid, label, unit, vs);
        if (n < 0 || (size_t)n >= capacity) { out_line[0] = '\0'; return false; }
    }
    return true;

emit_raw:
    /* For bitmask/hex PIDs that have no scalar value */
    {
        int n = snprintf(out_line, capacity, "PID,%02X,%s,%s,%s\r\n",
                         (unsigned)pid, label, unit, vs);
        if (n < 0 || (size_t)n >= capacity) { out_line[0] = '\0'; return false; }
    }
    return true;
}
