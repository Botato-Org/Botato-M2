/**
 * @file  tof.h
 * @brief Multi-instance VL53L0X driver (C): bring-up, addressing, continuous ranging, filtering.
 *
 * Per-device struct, so all five sensors run independently. Init sequence follows ST's API
 * (SPAD config, default tuning, VHV + phase calibration). Filter settings are in ToFConfig.h.
 */
#ifndef APP_TOF_H
#define APP_TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#include "ToFConfig.h"

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t            address;  // 8-bit I2C address, already shifted
    uint8_t            stop_variable;
    uint32_t           measurement_timing_budget_us;

    // ---- Filter state (per sensor) ------------------------------------------
    uint16_t window[TOF_MEDIAN_WINDOW];  // ring buffer of recent good raw reads
    uint8_t  window_count;               // valid samples collected so far
    uint8_t  window_index;               // next write position in the ring
    float    ema;                        // filtered mm
    bool     ema_valid;                  // false until the first good reading
    uint16_t last_raw_mm;                // most recent raw reading (pre-filter)
    uint8_t  last_status;                // range status of the most recent read
} VL53L0X_Dev_t;

/// Full bring-up, once per sensor after XSHUT release and before ranging.
/// addr_8bit is the sensor's CURRENT address (usually 0x29 << 1).
HAL_StatusTypeDef VL53L0X_Init(VL53L0X_Dev_t* dev, I2C_HandleTypeDef* hi2c, uint8_t addr_8bit);

/// Change the I2C address (already left-shifted) and update dev->address.
HAL_StatusTypeDef VL53L0X_SetAddress(VL53L0X_Dev_t* dev, uint8_t new_addr_8bit);

/// Blocking single-shot measurement in mm; HAL_TIMEOUT after ~100 ms. Used by bring-up tests.
HAL_StatusTypeDef VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev_t* dev, uint16_t* out_mm);

/// Integration time per reading (20000 fast ... 66000 accurate). Call before continuous mode.
HAL_StatusTypeDef VL53L0X_SetTimingBudget(VL53L0X_Dev_t* dev, uint32_t budget_us);

/// Start continuous ranging; period_ms = 0 means back-to-back at the timing budget.
HAL_StatusTypeDef VL53L0X_StartContinuous(VL53L0X_Dev_t* dev, uint32_t period_ms);

HAL_StatusTypeDef VL53L0X_StopContinuous(VL53L0X_Dev_t* dev);

/**
 * Read the latest continuous result through status gate -> median -> EMA; out_mm = filtered mm.
 *   HAL_OK      fresh good reading         HAL_BUSY     no new measurement yet (non-blocking)
 *   HAL_ERROR   bad status / out of range  HAL_TIMEOUT  I2C failure (bus fault, not range)
 */
HAL_StatusTypeDef VL53L0X_ReadRangeContinuousFiltered(VL53L0X_Dev_t* dev, uint16_t* out_mm);

#ifdef __cplusplus
}
#endif

#endif  // APP_TOF_H
