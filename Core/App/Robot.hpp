/**
 * @file  Robot.hpp
 * @brief Facade that owns every subsystem and wires them together.
 */
#ifndef APP_ROBOT_HPP
#define APP_ROBOT_HPP

#include "Indicator.hpp"
#include "Encoder.hpp"
#include "Gyro.hpp"
#include "WallSensorArray.hpp"
#include "DifferentialDrive.hpp"
#include "ControlLoop.hpp"
#include "ButtonManager.hpp"
#include "ModeController.hpp"
#include "Navigator.hpp"

/** Single global instance (`robot`); no dynamic allocation. */
class Robot {
public:
    void init();           // competition: initCore() + navigator + mode controller
    void initCore();       // hardware + control only (robot-based tests); starts TIM3
    void onControlTick();  // TIM3 ISR, 1 kHz: gyro, encoders, control loop
    void onMainLoop();     // competition main loop
    void serviceCore();    // ToF poll, buttons, LED: the part tests also need

    Encoder& encoderL() { return encL_; }
    Encoder& encoderR() { return encR_; }
    Gyro& gyro() { return gyro_; }
    WallSensorArray& walls() { return walls_; }
    DifferentialDrive& drive() { return drive_; }
    ControlLoop& control() { return ctrl_; }
    ButtonManager& buttons() { return btn_; }
    ModeController& modes() { return modes_; }
    Navigator& navigator() { return navigator_; }
    Indicator& led() { return led_; }

private:
    Encoder           encL_;
    Encoder           encR_;
    Gyro              gyro_;
    WallSensorArray   walls_;
    DifferentialDrive drive_;
    ControlLoop       ctrl_;
    ButtonManager     btn_;
    Indicator         led_;
    Navigator         navigator_;
    ModeController    modes_;
    volatile bool     coreReady_ = false;
};

extern Robot robot;

#endif  // APP_ROBOT_HPP
