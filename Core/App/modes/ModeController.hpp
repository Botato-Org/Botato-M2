/**
 * @file  ModeController.hpp
 * @brief Run lifecycle driven by the buttons; owns the LED policy and fault handling.
 */
#ifndef APP_MODECONTROLLER_HPP
#define APP_MODECONTROLLER_HPP

#include "ButtonManager.hpp"
#include "ControlLoop.hpp"
#include "Gyro.hpp"
#include "Navigator.hpp"
#include "Indicator.hpp"

/**
 * Buttons and LED patterns are listed in README §4; fault codes in DECISIONS #33.
 * One press = one event; pending events are dropped after acting, and presses are ignored for
 * RUN_GRACE_MS after a run starts so the start press can never abort it.
 */
class ModeController {
public:
    enum State : uint8_t { CALIBRATING, IDLE, SEARCH_RUN, SPEED_RUN, FAULT };
    enum Fault : uint8_t {
        NONE            = 0,
        GYRO_NOT_FOUND  = 2,
        GYRO_LOST       = 3,
        NO_CONTROL_TICK = 4
    };  // = flash count

    void init(ButtonManager* btn, ControlLoop* ctrl, Gyro* gyro, Navigator* nav, Indicator* led);
    void update();

    State state() const { return state_; }
    Fault fault() const { return fault_; }

private:
    ButtonManager* btn_  = nullptr;
    ControlLoop*   ctrl_ = nullptr;
    Gyro*          gyro_ = nullptr;
    Navigator*     nav_  = nullptr;
    Indicator*     led_  = nullptr;

    State    state_         = CALIBRATING;
    Fault    fault_         = NONE;
    uint32_t seenDecisions_ = 0;
    uint32_t calStartMs_    = 0;
    uint32_t refusedUntil_  = 0;  // 5-flash "no map" notice, until this time
    uint32_t speedRefused_  = 0;  // PA5 presses refused (no map yet)

    uint32_t runStartMs_ = 0;

    void calibrate();
    void idleLed();  // OFF (normal mode) or HEARTBEAT (curved mode)
    void enterFault(Fault f);
    void enterIdle();
    void startRun(State s);
};

#endif  // APP_MODECONTROLLER_HPP
