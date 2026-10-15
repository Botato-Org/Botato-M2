/**
 * @file  MotorConfig.h
 * @brief Motor dead-band.
 */
#ifndef APP_MOTORCONFIG_H
#define APP_MOTORCONFIG_H

namespace motorcfg {

// Non-zero commands are scaled into [DEADBAND, PWM_MAX].
constexpr int DEADBAND = 500;  // TUNE(TEST_MOTOR): raise if a wheel stalls, lower if jerky

}  // namespace motorcfg

#endif  // APP_MOTORCONFIG_H
