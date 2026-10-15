/**
 * @file  DifferentialDrive.hpp
 * @brief Owns both motors and mixes forward + turn into wheel PWM.
 */
#ifndef APP_DIFFERENTIALDRIVE_HPP
#define APP_DIFFERENTIALDRIVE_HPP

#include <cstdint>
#include "Motor.hpp"

/**
 * L = fwd - turn, R = fwd + turn. When saturated, forward is reduced so steering keeps
 * priority (DECISIONS #26). Takes float so no caller overflows before the clamp.
 */
class DifferentialDrive {
public:
    void init();

    void setPwm(float fwdPwm, float turnPwm);       // mixed, used by ControlLoop
    void setWheelPwm(int32_t left, int32_t right);  // direct, used by TEST_MOTOR
    void stop();

    int32_t lastLeftPwm() const { return lastLeft_; }
    int32_t lastRightPwm() const { return lastRight_; }

private:
    Motor   left_;
    Motor   right_;
    int32_t lastLeft_  = 0;
    int32_t lastRight_ = 0;
};

#endif  // APP_DIFFERENTIALDRIVE_HPP
