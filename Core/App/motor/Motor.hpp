/**
 * @file  Motor.hpp
 * @brief One TB6612 channel: a PWM timer channel and two direction pins.
 */
#ifndef APP_MOTOR_HPP
#define APP_MOTOR_HPP

#include <cstdint>
extern "C" {
#include "tim.h"
#include "gpio.h"
}

/** Direction polarity is inverted to match the wiring (DECISIONS #15); applies the dead-band. */
class Motor {
public:
    void init(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, GPIO_TypeDef* in1Port,
              uint16_t in1Pin, GPIO_TypeDef* in2Port, uint16_t in2Pin);

    // pwm in [-PWM_MAX, +PWM_MAX], clamped here. Positive = physical forward.
    void setPwm(int32_t pwm);
    void stop();  // coast: both direction pins low

private:
    TIM_HandleTypeDef* pwmTim_     = nullptr;
    uint32_t           pwmChannel_ = 0;
    GPIO_TypeDef*      in1Port_    = nullptr;
    uint16_t           in1Pin_     = 0;
    GPIO_TypeDef*      in2Port_    = nullptr;
    uint16_t           in2Pin_     = 0;
};

#endif  // APP_MOTOR_HPP
