/**
 * @file  Gyro.hpp
 * @brief MPU6050 Z-axis yaw on the dedicated I2C2 bus (DECISIONS #16).
 */
#ifndef APP_GYRO_HPP
#define APP_GYRO_HPP

#include <cstdint>
extern "C" {
#include "i2c.h"
}

/**
 * After init(), every I2C2 transaction (including bias calibration) runs in update() in the
 * 1 kHz ISR. Heading is anticlockwise-positive, continuous and never wrapped.
 */
class Gyro {
public:
    bool init(I2C_HandleTypeDef* hi2c);  // blocking, main context, once
    void beginCalibration();             // non-blocking; robot must be still
    void update();                       // 1kHz ISR

    bool ok() const { return ok_; }
    bool lost() const { return lost_; }            // stopped answering after init
    uint32_t updates() const { return updates_; }  // ISR calls: proves TIM3 runs
    bool calibrating() const { return calibrating_; }
    bool calibrated() const { return calibrated_; }
    float rateDps() const { return rateDps_; }
    float angleDeg() const { return angleDeg_; }
    float bias() const { return bias_; }
    uint8_t whoAmI() const { return whoAmI_; }
    uint32_t readErrors() const { return readErrors_; }

    void zeroAngle() { angleDeg_ = 0.0f; }  // call inside a CriticalSection

private:
    I2C_HandleTypeDef* hi2c_        = nullptr;
    bool               ok_          = false;
    volatile bool      calibrating_ = false;
    bool               calibrated_  = false;
    uint8_t            whoAmI_      = 0;
    float              bias_        = 0.0f;
    float              rateDps_     = 0.0f;
    float              angleDeg_    = 0.0f;
    float              calSum_      = 0.0f;
    int                calTicks_    = 0;
    uint32_t           readErrors_  = 0;
    uint32_t           updates_     = 0;
    int                failStreak_  = 0;
    bool               lost_        = false;

    bool readRawZ(int16_t& out) const;
};

#endif  // APP_GYRO_HPP
