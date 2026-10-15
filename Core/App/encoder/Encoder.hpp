/**
 * @file  Encoder.hpp
 * @brief One 32-bit hardware quadrature timer: distance and speed.
 */
#ifndef APP_ENCODER_HPP
#define APP_ENCODER_HPP

#include <cstdint>
#include "EncoderConfig.h"
extern "C" {
#include "tim.h"
}

/** Right wheel is negated so forward is positive on both (DECISIONS #18). update(): 1 kHz ISR. */
class Encoder {
public:
    void init(TIM_HandleTypeDef* htim, bool invert);
    void update();  // 1kHz ISR

    int32_t count() const { return count_; }            // signed, since init
    int32_t deltaTicks() const { return delta_; }       // this tick
    float distanceMm() const { return distanceMm_; }    // since init
    float speedMmPerS() const { return speedMmPerS_; }  // windowed average

private:
    TIM_HandleTypeDef* htim_        = nullptr;
    bool               invert_      = false;
    int32_t            count_       = 0;
    int32_t            delta_       = 0;
    float              distanceMm_  = 0.0f;
    float              speedMmPerS_ = 0.0f;

    int32_t window_[enccfg::SPEED_WINDOW] = {};
    int32_t windowSum_                    = 0;
    int     windowIdx_                    = 0;

    int32_t readCounter() const;
};

#endif  // APP_ENCODER_HPP
