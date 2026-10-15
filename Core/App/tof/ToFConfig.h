/**
 * @file  ToFConfig.h
 * @brief VL53L0X timing, filter and per-sensor calibration.
 *
 * C part (macros) is read by tof.c; the C++ part by WallSensorArray and the tests.
 */
#ifndef APP_TOFCONFIG_H
#define APP_TOFCONFIG_H

// ---- Timing and filter: status gate -> median -> EMA ------------------------
#define TOF_TIMING_BUDGET_US 20000u  // us · sensor minimum, ~45 samples/s; 33000 = less noise
#define TOF_MEDIAN_WINDOW    3       // rejects single spikes, one sample of lag
#define TOF_EMA_ALPHA        0.6f

#ifdef __cplusplus
#include <cstdint>

namespace tofcfg {

constexpr uint32_t TIMING_BUDGET_US = TOF_TIMING_BUDGET_US;
constexpr uint32_t SAMPLE_PERIOD_MS = TOF_TIMING_BUDGET_US / 1000u;
constexpr uint32_t POLL_MARGIN_MS   = 2;                     // ms · added before the next poll
constexpr uint32_t STALE_MS         = 4 * SAMPLE_PERIOD_MS;  // no valid sample -> no-target
constexpr float    NO_TARGET_MM     = 8190.0f;               // far sentinel, never 0 (#27)

// ---- XSHUT boot sequencing (index order = cfg::ToFIndex) --------------------
constexpr uint8_t  ADDR[5]             = {0x30, 0x31, 0x32, 0x33, 0x34};
constexpr uint32_t XSHUT_BOOT_DELAY_MS = 10;

// ---- Calibration: true mm from the lens = CAL_A · filtered + CAL_B ----------
// TUNE(TEST_TOF_CAL): tofCal.fitA / fitB. Index 0 L, 1 LF, 2 F, 3 RF, 4 R.
// Worst-point residual (mm): L 2.1, LF 3.6, F 3.0, RF 2.8, R 2.2.
constexpr float CAL_A[5] = {0.921611f, 0.942740f, 0.920044f, 0.925280f, 0.982126f};
constexpr float CAL_B[5] = {-18.6685f, -20.2615f, -13.8093f, -1.9881f, -20.7765f};

}  // namespace tofcfg
#endif  // __cplusplus

#endif  // APP_TOFCONFIG_H
