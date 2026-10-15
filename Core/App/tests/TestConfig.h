/**
 * @file  TestConfig.h
 * @brief Parameters for the test build only (see main_test.cpp).
 */
#ifndef APP_TESTCONFIG_H
#define APP_TESTCONFIG_H

#include <cstdint>
#include "Config.h"

namespace testcfg {

// ---- TEST_TOF_SINGLE --------------------------------------------------------
// One breakout gives a single ~10k pull-up: try 100000 if busErrors climb. 0 = keep the .ioc.
constexpr uint32_t TOF1_FORCE_BUS_HZ = 0;

// ---- TEST_TOF_CAL -----------------------------------------------------------
constexpr int   CAL_POINTS                = 6;
constexpr float CAL_POINTS_MM[CAL_POINTS] = {30.0f,  50.0f,  80.0f,
                                             120.0f, 160.0f, 200.0f};  // lens face
constexpr int   CAL_SAMPLES               = 40;                        // averaged per point (~1 s)

// ---- TEST_ENCODER -----------------------------------------------------------
constexpr float ENC_PUSH_MM = 500.0f;  // push exactly this far along a ruler

// ---- TEST_MOTOR (wheels off the ground) -------------------------------------
constexpr int32_t  MOTOR_TEST_PWM = 1500;  // open-loop PWM per phase
constexpr uint32_t MOTOR_PHASE_MS = 1500;

// ---- TEST_TURN: cycles of L90, R90, R90, L90, 180, 180 ----------------------
constexpr int      TURN_CYCLES   = 3;
constexpr uint32_t TURN_PAUSE_MS = 400;

// ---- TEST_CURVE: lead-in, one curve at CURVE_SPEED_MMPS, lead-out (real mm) ---
constexpr float CURVE_LEAD_IN_MM  = 300.0f;
constexpr float CURVE_LEAD_OUT_MM = 300.0f;

// ---- TEST_DRIVE_CELLS -------------------------------------------------------
constexpr int   DRIVE_CELLS      = 15;
constexpr float DRIVE_SPEED_MMPS = cfg::SEARCH_SPEED_MMPS;

}  // namespace testcfg

#endif  // APP_TESTCONFIG_H
