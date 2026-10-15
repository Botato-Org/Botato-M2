/**
 * @file  Critical.hpp
 * @brief Scoped interrupt mask for multi-field updates shared with the TIM3 ISR.
 */
#ifndef APP_CRITICAL_HPP
#define APP_CRITICAL_HPP

#include <cstdint>
extern "C" {
#include "main.h"
}

/** Masks interrupts for its lifetime; restores PRIMASK, so nesting is safe. Keep scopes tiny. */
class CriticalSection {
public:
    CriticalSection() : primask_(__get_PRIMASK()) { __disable_irq(); }
    ~CriticalSection() { __set_PRIMASK(primask_); }
    CriticalSection(const CriticalSection&)            = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

private:
    uint32_t primask_;
};

#endif  // APP_CRITICAL_HPP
