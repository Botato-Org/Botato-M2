/**
 * @file  GyroConfig.h
 * @brief MPU6050 register values, calibration length and loss detection.
 */
#ifndef APP_GYROCONFIG_H
#define APP_GYROCONFIG_H

#include <cstdint>

namespace gyrocfg {

constexpr uint8_t  ADDR_7B         = 0x68;   // AD0 tied to GND
constexpr uint8_t  GYRO_CONFIG_VAL = 0x08;   // FS_SEL=1 -> +/-500 dps
constexpr float    LSB_PER_DPS     = 65.5f;  // matches FS_SEL=1
constexpr uint8_t  DLPF_VAL        = 0x03;   // ~44 Hz
constexpr int      OVERSAMPLE      = 1;      // output register updates at 1 kHz: one read per tick
constexpr uint32_t READ_TIMEOUT_MS = 2;

constexpr int CAL_TICKS  = 5000;  // ticks · bias calibration, robot must stay still
constexpr int LOST_TICKS = 100;   // consecutive failed reads = gyro lost -> fault

}  // namespace gyrocfg

#endif  // APP_GYROCONFIG_H
