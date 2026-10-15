/**
 * @file  ControlLoop.hpp
 * @brief 1 kHz controller: speed ramp + heading/rate cascade with feed-forward (DECISIONS #23).
 */
#ifndef APP_CONTROLLOOP_HPP
#define APP_CONTROLLOOP_HPP

#include <cstdint>
#include "Encoder.hpp"
#include "Gyro.hpp"
#include "DifferentialDrive.hpp"
#include "PIDController.hpp"
#include "MotionProfile.hpp"

/**
 * Commands come from the main loop; tick() runs in the TIM3 ISR.
 * Heading is anticlockwise-positive and never wrapped, so turnBy(+180) is unambiguous (#22).
 */
class ControlLoop {
public:
    void init(Gyro* gyro, Encoder* encL, Encoder* encR, DifferentialDrive* drive);

    void enable(bool on);  // off: motors stopped, commands zeroed
    bool enabled() const { return enabled_; }
    void resetForRun();  // zero heading, profiles and PIDs

    // ---- Commands (main loop) -----------------------------------------------
    void setForwardSpeed(float mmps);
    void turnBy(float deltaDeg);
    void holdHeading();                   // leave raw mode, hold the current goal
    void setHeadingTrim(float deg);       // wall-centring lean, clamped
    void setRate(float mmps, float dps);  // raw rate mode for tests; bypasses the heading loop

    bool turnDone() const;

    /// Curved turn while driving: heading follows ArcShape over distance, forward speed unchanged.
    void arcTurn(float deltaDeg, float lengthMm);
    bool arcActive() const { return arcActive_; }

    // ---- 1 kHz ISR ----------------------------------------------------------
    void tick();

    // ---- Telemetry ----------------------------------------------------------
    float pvX() const { return pvX_; }
    float pvW() const { return pvW_; }
    float speedRef() const { return ramp_.value(); }
    float speedCmd() const { return vCmd_; }
    float headingDeg() const { return gyro_->angleDeg(); }
    float headingRef() const { return heading_.pos(); }
    float headingGoal() const { return heading_.goal(); }
    float rateRef() const { return heading_.vel(); }
    float headingErr() const { return headingErr_; }
    float trimDeg() const { return trim_; }
    float arcRateFfDps() const { return arcFfDps_; }
    float arcMaxLagDeg() const { return arcMaxLagDeg_; }  // worst heading lag in the last curve
    float arcEndErrDeg() const { return arcEndErrDeg_; }  // heading error when it ended

private:
    Gyro*              gyro_  = nullptr;
    Encoder*           encL_  = nullptr;
    Encoder*           encR_  = nullptr;
    DifferentialDrive* drive_ = nullptr;

    volatile bool enabled_    = false;
    bool          rawMode_    = false;
    float         vCmd_       = 0.0f;
    float         rawRateCmd_ = 0.0f;
    float         trim_       = 0.0f;
    uint32_t      turnTicks_  = 0;

    // curved turn state (written under CriticalSection, run in the ISR)
    volatile bool arcActive_   = false;
    float         arcStartDeg_ = 0.0f;  // heading at the start of the curve
    float         arcDeltaDeg_ = 0.0f;  // +90 left, -90 right
    float         arcLenMm_    = 0.0f;  // curve length along the axle path
    float         arcS0Mm_     = 0.0f;  // axle distance when the curve started
    float         arcFfDps_    = 0.0f;  // planned turn rate right now
    float         arcFfPwmDps_ = 0.0f;  // rate the motors are driven for (gain, lead applied)
    float arcMaxLagDeg_ = 0.0f;  // diagnostics: + = heading behind the plan (curve going wide)
    float arcEndErrDeg_ = 0.0f;

    SpeedRamp    ramp_;
    AngleProfile heading_;

    float pvX_ = 0.0f, pvW_ = 0.0f, headingErr_ = 0.0f;

    PIDController pidSpeed_{ctrlcfg::SPEED};
    PIDController pidRate_{ctrlcfg::RATE};
    PIDController pidHeading_{ctrlcfg::HEADING};
};

#endif  // APP_CONTROLLOOP_HPP
