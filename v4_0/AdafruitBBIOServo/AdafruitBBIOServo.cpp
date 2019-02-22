/*
 * AdafruitBBIOServo.cpp
 *
 * BBIO C++ Library controls Beaglebone's Servo motor by PWM.
 *
 *  Created on: FEb 22, 2019
 *  Author: Chao Li
 *  Email:forward.li.kth@gmail.com
 *  Blog: http://forwardkth.github.io/
 */

#include "AdafruitBBIOServo.h"

namespace adafruit {
namespace bbio {


AdafruitBBIOServo::AdafruitBBIOServo(std::string const& pwmPin)
    : pwm(pwmPin),
      max_ms(2.4),
      min_ms(0.5),
      pwmFreq(50),
      period(1000.0 / pwmFreq) {
  //pwm.start(0.0, 2000.0, Pwm::Polarity::Normal);
  pwm.start(0., 1e9 / (1 * 1000 * 1000), Pwm::Polarity::Normal);
}

AdafruitBBIOServo::~AdafruitBBIOServo() {
  // TODO Auto-generated destructor stub
  this->ReleasePWM();
}

bool AdafruitBBIOServo::write_angle(int angle) {
  pwm.start(0., 1e9 / (1 * 1000 * 1000), Pwm::Polarity::Normal);
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;
  float load_time = (max_ms - min_ms) / 180.0 * angle + min_ms;
  float duty_cycle =load_time /(period - load_time) * 100;
  pwm.set_frequency(pwmFreq);
  pwm.set_duty_cycle(duty_cycle);
  return true;
}

bool AdafruitBBIOServo::ReleasePWM() {
  pwm.stop();
  return true;
}

} /* namespace BlackLib */
}
