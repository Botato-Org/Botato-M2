/**
 * @file  ButtonConfig.h
 * @brief Button sampling, debounce and long-press timing.
 */
#ifndef APP_BUTTONCONFIG_H
#define APP_BUTTONCONFIG_H

#include <cstdint>

namespace btncfg {

constexpr uint32_t SAMPLE_MS    = 5;     // ms · raw sampling interval
constexpr uint32_t DEBOUNCE_MS  = 20;    // ms · new level must hold this long
constexpr uint32_t LONGPRESS_MS = 3000;  // ms · PA6 recalibrate, PA5 toggle curves

}  // namespace btncfg

#endif  // APP_BUTTONCONFIG_H
