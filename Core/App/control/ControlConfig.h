/**
 * @file  ControlConfig.h
 * @brief ControlLoop gains, feed-forward, output caps and turn completion.
 */
#ifndef APP_CONTROLCONFIG_H
#define APP_CONTROLCONFIG_H

#include <cstdint>
#include "Config.h"

namespace ctrlcfg {

struct Gains {
    float kp, ki, kd, iClamp;
};

// ---- PID gains --------------------------------------------------------------
constexpr Gains SPEED   = {4.0f, 0.15f, 0.0f, 400.0f};   // mm/s error -> fwd PWM
constexpr Gains RATE    = {10.0f, 0.0f, 25.0f, 400.0f};  // deg/s error -> turn PWM
constexpr Gains HEADING = {8.0f, 0.0f, 5.0f, 200.0f};    // deg error -> deg/s

// ---- Forward feed-forward (DECISIONS #23) -----------------------------------
constexpr float SPEED_FF = cfg::PWM_MAX / cfg::FREE_RUN_MMPS;  // TUNE(TEST_MOTOR): suggestedSpeedFF
constexpr float MOTOR_TAU_S = 0.040f;                  // TUNE(TEST_MOTOR): motor lag, tauMs / 1000
constexpr float ACCEL_FF    = SPEED_FF * MOTOR_TAU_S;  // cancels the lag while ramping

// ---- Output caps ------------------------------------------------------------
constexpr float HEADING_CORR_MAX_DPS = 120.0f;   // deg/s · heading correction on top of profile FF
constexpr float MAX_TURN_PWM_DRIVING = 1200.0f;  // keeps forward PWM headroom while steering
constexpr float MAX_HEADING_TRIM_DEG = 12.0f;    // deg · largest wall-centring lean

// ---- Curved turns -----------------------------------------------------------
constexpr float ARC_RAMP_FRAC = 0.25f;  // fraction of the curve spent ramping rate up / down
// Turn PWM per deg/s while driving: wheel-speed difference (WHEELBASE/2 · rad) × SPEED_FF.
constexpr float ARC_RATE_FF      = SPEED_FF * (cfg::WHEELBASE_MM * 0.5f) * (cfg::PI_F / 180.0f);
constexpr float MAX_TURN_PWM_ARC = 3000.0f;  // turn-PWM cap in a curve (~1100 used at 300 mm/s)
constexpr float ARC_TURN_FF_GAIN = 1.70f;    // TUNE(TEST_CURVE): > 1 tighter, < 1 wider
constexpr float ARC_TURN_LEAD_S  = 0.10f;    // TUNE(TEST_CURVE): leads the turn by motor lag

// ---- Turn completion --------------------------------------------------------
// Done when the profile has finished and the robot has settled, or on timeout.
constexpr float    TURN_TOL_DEG    = 2.0f;
constexpr float    TURN_SETTLE_DPS = 20.0f;
constexpr uint32_t TURN_TIMEOUT_MS = 3000;

}  // namespace ctrlcfg

#endif  // APP_CONTROLCONFIG_H
