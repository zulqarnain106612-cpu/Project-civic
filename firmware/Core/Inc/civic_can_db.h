#ifndef CIVIC_CAN_DB_H
#define CIVIC_CAN_DB_H
/*
 * civic_can_db.h — Honda Civic 9th/10th-gen native CAN frame lookup table.
 *
 * IDs sourced from opendbc (Honda community), cross-referenced against
 * observed frames from this vehicle's CAN analyzer capture.
 * Labels are best-known guesses; byte-level signals are confirmed by
 * observing which bytes are variable at runtime (can_signal_profiler).
 *
 * Transceiver: TJA1051 (NXP) — only supported CAN transceiver.
 * No substitutes.
 */

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t    id;
    uint8_t     is_extended;          /* 1 = 29-bit, 0 = 11-bit */
    const char *frame_label;
    /* Per-byte labels; NULL means byte purpose unknown */
    const char *byte_labels[8];
} civic_can_entry_t;

/* ---------- 11-bit standard frame database ----------------------------- */
static const civic_can_entry_t civic_can_db[] = {

    /* ── Engine / powertrain ────────────────────────────────────────── */
    { 0x17C, 0, "Engine Speed / Throttle / Ignition",
      { "RPM_H",  "RPM_L",    "Throttle%", "EngLoad%",
        "IgnAdv", "SparkAdv", NULL,         NULL } },
    { 0x0C4, 0, "Engine Speed 2 / Aux",
      { "RPM2_H", "RPM2_L",  "Idle",      NULL,
        NULL,     NULL,       NULL,         NULL } },
    { 0x08F, 0, "PCM Status / Check Engine",
      { "PCMState", "CheckEng", "ReadyCodes", "MIL_dist_H",
        "MIL_dist_L", NULL, NULL, NULL } },

    /* ── Air intake / MAP / MAF / injectors ────────────────────────── */
    { 0x0A5, 0, "MAF / MAP / IAT / Injectors",
      { "MAF_H",   "MAF_L",   "MAP_kPa",  "IAT_raw",
        "ECT_raw", "InjPW_H", "InjPW_L",  NULL } },

    /* ── Fuel system / injection / O2 sensors ───────────────────────── */
    { 0x0D9, 0, "Fuel System / Injection / O2",
      { "FuelStat",   "FuelTrim_ST", "FuelTrim_LT", "O2_B1S1_V",
        "InjDuty",    "LambdaCtrl",  "O2_B1S2_V",   NULL } },
    { 0x2B2, 0, "Fuel Level / Tank",
      { "FuelLevel",  "FuelLevel2", NULL,  NULL,
        NULL,          NULL,          NULL,  NULL } },
    { 0x0F3, 0, "Engine Control Unit 2",
      { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL } },

    /* ── Exhaust / emissions ────────────────────────────────────────── */
    { 0x0B6, 0, "Vehicle Speed 2 / Exhaust Emissions",
      { "VehSpd_H", "VehSpd_L", "EGR_cmd", "EGR_err",
        "CatTemp_H", "CatTemp_L", NULL,     NULL } },

    /* ── Transmission / CVT ────────────────────────────────────────── */
    { 0x1B0, 0, "Transmission / CVT",
      { "GearPos",      "TCC",          "InputSpd_H",  "InputSpd_L",
        "OutputSpd_H",  "OutputSpd_L",  NULL,           NULL } },
    { 0x300, 0, "Gear / Shift Position",
      { "GearRaw",  "ShiftMode", NULL,        NULL,
        "GearDisp", NULL,        NULL,         NULL } },
    { 0x3E0, 0, "Transmission Control 2",
      { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL } },

    /* ── Wheel speeds / brakes / dynamics ───────────────────────────── */
    { 0x158, 0, "Wheel Speeds FL/FR/RL/RR",
      { "FL_H", "FL_L", "FR_H", "FR_L", "RL_H", "RL_L", "RR_H", "RR_L" } },
    { 0x1A0, 0, "Brake Pressure / Deceleration",
      { "BrkPress_H", "BrkPress_L", "Decel_H", "Decel_L",
        NULL,          NULL,          NULL,       NULL } },
    { 0x0DC, 0, "VSA / Stability Control",
      { "YawRate_H",  "YawRate_L",  "LatAccel_H",  "LatAccel_L",
        "LongAccel_H","LongAccel_L", NULL,           NULL } },

    /* ── Steering ──────────────────────────────────────────────────── */
    { 0x0E4, 0, "Steering Angle / Torque",
      { "SteerAngle_H", "SteerAngle_L", "SteerRate_H", "SteerRate_L",
        "Torque_H",      "Torque_L",     NULL,           NULL } },

    /* ── ADAS / LKAS ────────────────────────────────────────────────── */
    { 0x19E, 0, "ADAS / LKAS",
      { "LKASState", "Torque_H", "Torque_L", "LaneConf",
        NULL,         NULL,        NULL,       NULL } },

    /* ── Body / comfort ─────────────────────────────────────────────── */
    { 0x0CE, 0, "Body Control Module",
      { "DoorStat", "LockStat", "WinStat", "LightStat",
        NULL,        NULL,       NULL,       NULL } },
    { 0x0BF, 0, "HVAC / Climate Control",
      { "FanSpd", "SetTemp", "Mode",   "AirDir",
        "AcReq",  "DefReq",  NULL,      NULL } },
    { 0x0C0, 0, "Ambient Sensor / Light",
      { "AmbLight", "AmbTemp", NULL, NULL, NULL, NULL, NULL, NULL } },
    { 0x1FB, 0, "BCM Power / Battery Voltage",
      { "BatVolt_H", "BatVolt_L", NULL, NULL, NULL, NULL, NULL, NULL } },
    { 0x4A9, 0, "Infotainment Bus",
      { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL } },
    { 0x501, 0, "Gateway / Diagnostic",
      { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL } },

    /* ── OBD-II functional addresses ────────────────────────────────── */
    { 0x7DF, 0, "OBD-II Broadcast Request",
      { "Mode", "PID", NULL, NULL, NULL, NULL, NULL, NULL } },
    { 0x7E0, 0, "OBD-II ECM Request (Honda)",
      { "Mode", "PID", NULL, NULL, NULL, NULL, NULL, NULL } },
    { 0x7E8, 0, "OBD-II ECM Response",
      { "Len", "Mode_Resp", "PID", "A", "B", "C", "D", NULL } },
};

#define CIVIC_CAN_DB_COUNT ((uint32_t)(sizeof(civic_can_db) / sizeof(civic_can_db[0])))

/*
 * civic_can_db_lookup() — return entry for given ID, or NULL if not in table.
 */
static inline const civic_can_entry_t *civic_can_db_lookup(uint32_t id, uint8_t extended)
{
    for (uint32_t i = 0u; i < CIVIC_CAN_DB_COUNT; i++) {
        if (civic_can_db[i].id == id && civic_can_db[i].is_extended == extended)
            return &civic_can_db[i];
    }
    return NULL;
}

#endif /* CIVIC_CAN_DB_H */
