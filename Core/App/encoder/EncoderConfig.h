/**
 * @file  EncoderConfig.h
 * @brief Encoder speed-estimate window.
 */
#ifndef APP_ENCODERCONFIG_H
#define APP_ENCODERCONFIG_H

namespace enccfg {

constexpr int SPEED_WINDOW = 4;  // ticks · ~25 mm/s resolution for 2 ms of lag

}  // namespace enccfg

#endif  // APP_ENCODERCONFIG_H
