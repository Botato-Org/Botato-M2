/**
 * @file  Indicator.hpp
 * @brief Non-blocking status LED; update() every main-loop pass.
 */
#ifndef APP_INDICATOR_HPP
#define APP_INDICATOR_HPP

#include <cstdint>

/**
 * BLINK calibrating, OFF idle/running, pulse() per decision, code(n) fault, HEARTBEAT idle in
 * curved mode, solid ON Error_Handler. Test builds: PASS = code(2), FAIL = code(3).
 */
class Indicator {
public:
    enum Pattern { OFF, ON, BLINK, PASS, FAIL, CODE, RAPID, HEARTBEAT };

    void init();
    void set(Pattern p);
    void code(int flashes);  // fault code: n flashes, pause, repeat
    void pulse();            // one flash over the current pattern
    void update();

    Pattern pattern() const { return pattern_; }

private:
    Pattern  pattern_    = OFF;
    uint32_t lastMs_     = 0;
    int      step_       = 0;
    int      flashes_    = 0;
    bool     pulsing_    = false;
    uint32_t pulseStart_ = 0;
    void write(bool on);
};

#endif  // APP_INDICATOR_HPP
