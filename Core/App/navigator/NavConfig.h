/**
 * @file  NavConfig.h
 * @brief Navigator thresholds: decision point, wall detection, stops, parking, curves, centring.
 *
 * Every decision and turn happens with the wheel axle on a cell centre (DECISIONS #25).
 * Distances are calibrated mm from each ToF lens (after ToFConfig.h).
 */
#ifndef APP_NAVCONFIG_H
#define APP_NAVCONFIG_H

#include <cstdint>
#include "Config.h"
#include "ControlConfig.h"

namespace navcfg {

// ---- Start ------------------------------------------------------------------
// Parked with the tail on the back wall: the axle is this far short of the start-cell centre.
constexpr float START_OFFSET_MM = cfg::CORRIDOR_MM * 0.5f - cfg::AXLE_TO_REAR_MM;  // 60.5

// ---- Decision point ---------------------------------------------------------
// Side sensors are exactly over the target cell centre, clear of the corner posts.
constexpr float SENSE_LOOKAHEAD_MM = cfg::TOF_SIDE_AHEAD_MM;  // 30

// ---- Wall detection ---------------------------------------------------------
// Present when the reading is BELOW the threshold. Check live with TEST_TOF_LIVE.
constexpr float SIDE_WALL_PRESENT_MM   = 120.0f;  // TUNE(TEST_TOF_LIVE): wall ~60, none ~250
constexpr float FRONT_STOP_MM          = 44.0f;   // TUNE(TEST_TOF_LIVE): front reading, centred
constexpr float FRONT_WALL_EXPECTED_MM = FRONT_STOP_MM + SENSE_LOOKAHEAD_MM;  // ~74
constexpr float FRONT_WALL_PRESENT_MM  = 150.0f;  // TUNE(TEST_TOF_LIVE): wall ~74, none ~266

static_assert(SIDE_WALL_PRESENT_MM > 90.0f && SIDE_WALL_PRESENT_MM < 220.0f,
              "SIDE_WALL_PRESENT_MM must sit between a side wall (~60) and no wall (~250)");
static_assert(FRONT_WALL_PRESENT_MM > FRONT_WALL_EXPECTED_MM + 30.0f &&
                  FRONT_WALL_PRESENT_MM < FRONT_WALL_EXPECTED_MM + cfg::CELL_PITCH_MM - 30.0f,
              "FRONT_WALL_PRESENT_MM must sit between a front wall and the next wall back");

// ---- Start-corner detection (DECISIONS #30) ---------------------------------
constexpr float OUTER_OPEN_MM = 200.0f;  // valid side reading beyond this = outer wall is open

// ---- Front-wall stop --------------------------------------------------------
constexpr float FRONT_REF_MAX_MM = 120.0f;  // only re-anchor on a front wall this close
static_assert(FRONT_REF_MAX_MM > FRONT_WALL_EXPECTED_MM + 20.0f,
              "FRONT_REF_MAX_MM must cover the front wall seen from the decision point");
// Pipeline lag: half a 20 ms sample + median-of-3 + EMA ~= 47 ms, compensated as speed × lag.
constexpr float FRONT_LATENCY_S = 0.047f;  // TUNE(maze): raise if fast stops end too close

// ---- Braking: v = sqrt(2 · decel · remaining) -------------------------------
constexpr float    BRAKE_DECEL_MMPS2 = 0.8f * cfg::DECEL_MMPS2;  // below the ramp limit
constexpr float    CREEP_MMPS        = 40.0f;
constexpr float    CREEP_ZONE_MM     = 6.0f;  // last mm covered at creep speed
constexpr float    STOP_TOL_MM       = 1.0f;
constexpr float    SETTLE_MMPS       = 15.0f;
constexpr uint32_t SETTLE_TIMEOUT_MS = 400;
constexpr float    PARK_SPEED_MMPS   = 150.0f;

// ---- Stall guard ------------------------------------------------------------
// No STALL_PROGRESS_MM for STALL_MS counts as arrived: within STALL_ARRIVE_MM on a stop,
// anywhere while parking (plus PARK_TIMEOUT_MS), so a stalled creep can't hang the run.
constexpr uint32_t STALL_MS          = 300;
constexpr float    STALL_PROGRESS_MM = 0.5f;
constexpr float    STALL_ARRIVE_MM   = 10.0f;
constexpr uint32_t PARK_TIMEOUT_MS   = 4000;
constexpr float    PARK_GAP_MM       = 8.0f;  // stop short of the back wall; next run allows for it

static_assert(cfg::SEARCH_SPEED_MMPS * cfg::SEARCH_SPEED_MMPS <=
                  2.0f * BRAKE_DECEL_MMPS2 * (SENSE_LOOKAHEAD_MM - CREEP_ZONE_MM),
              "SEARCH_SPEED_MMPS too high to brake within SENSE_LOOKAHEAD_MM: "
              "lower the speed or raise DECEL_MMPS2 in Config.h");

// ---- Curved turns -----------------------------------------------------------
// A curve runs from half a cell before the turn-cell centre to half a cell after it.
constexpr float ARC_DECIDE_MARGIN_MM = 20.0f;  // decide this much before the curve start
constexpr float ARC_START_ADVANCE_MM = 0.0f;   // TUNE(maze): + if curves start late, - if early
static_assert(ARC_START_ADVANCE_MM > -40.0f && ARC_START_ADVANCE_MM < 60.0f,
              "ARC_START_ADVANCE_MM out of range: expected a few mm to a few tens of mm");
// Front ToF sets the curve start when the cell has a known front wall; gated against encoders.
constexpr float ARC_FRONT_REF_MAX_MM  = 300.0f;
constexpr float ARC_FRONT_REF_GATE_MM = 60.0f;
static_assert(cfg::CURVE_SPEED_MMPS > 0.0f, "CURVE_SPEED_MMPS must be positive");

// ---- Lateral wall centring (DECISIONS #28) ----------------------------------
constexpr float WC_SUM_CENTERED_MM   = 122.4f;  // TUNE(TEST_TOF_LIVE): tofLive.sideSum
constexpr float WC_CENTER_TRIM_MM    = -4.9f;   // TUNE(TEST_TOF_LIVE): tofLive.sideDiff
constexpr float WC_LEFT_CENTERED_MM  = 0.5f * (WC_SUM_CENTERED_MM + WC_CENTER_TRIM_MM);  // ~58.8
constexpr float WC_RIGHT_CENTERED_MM = 0.5f * (WC_SUM_CENTERED_MM - WC_CENTER_TRIM_MM);  // ~63.7
constexpr float WC_WIDTH_TOL_MM      = 15.0f;  // both-wall sanity window
constexpr float WC_DEADBAND_MM       = 2.0f;
constexpr float WC_POST_GATE_MM      = 25.0f;  // ignore side walls this close to a post
// Wall-end protection: out of band or a jump longer latches that side off until the next cell.
constexpr float          WC_BAND_MM         = 25.0f;
constexpr float          WC_JUMP_MM         = 10.0f;
constexpr float          WC_GAIN_BOTH_WALLS = 1.00f;
constexpr float          WC_GAIN_ONE_WALL   = 0.75f;
constexpr ctrlcfg::Gains WALL_PID           = {0.75f, 0.004f, 0.55f, 600.0f};  // mm -> deg

}  // namespace navcfg

#endif  // APP_NAVCONFIG_H
