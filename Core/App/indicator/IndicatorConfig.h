/**
 * @file  IndicatorConfig.h
 * @brief Status LED pin and pattern timing.
 */
#ifndef APP_INDICATORCONFIG_H
#define APP_INDICATORCONFIG_H

#include <cstdint>
extern "C" {
#include "gpio.h"
}

namespace ledcfg {

inline GPIO_TypeDef* const PORT = GPIOC;  // Black Pill onboard LED, active LOW
constexpr uint16_t         PIN  = GPIO_PIN_13;

constexpr uint32_t BLINK_MS        = 500;   // ms · calibrating toggle
constexpr uint32_t RAPID_MS        = 80;    // ms · rapid flicker
constexpr uint32_t HEART_ON_MS     = 120;   // ms · idle in curved mode: short flash ...
constexpr uint32_t HEART_PERIOD_MS = 1000;  // ms · ... once a second
constexpr uint32_t PULSE_MS        = 80;    // ms · one flash per navigation decision
constexpr uint32_t PATTERN_MS      = 150;   // ms · step time for flash codes

}  // namespace ledcfg

#endif  // APP_INDICATORCONFIG_H
