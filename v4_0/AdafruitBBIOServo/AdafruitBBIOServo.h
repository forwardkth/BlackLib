// AdafruitBBIOServo.h
// BBIO C++ Library controls Beaglebone's Servo motor by PWM.
// Created on: Feb 22, 2010
// Author: Chao Li
// Email:forward.li.kth@gmail.com
// Blog: http://forwardkth.github.io/

#ifndef ADAFRUIT_BBIO_SERVO_H_
#define ADAFRUIT_BBIO_SERVO_H_

#include <chrono>
#include "../AdafruitBBIOLib/bbio.h"

namespace adafruit {
namespace bbio {

class AdafruitBBIOServo {
 public:
  /*! @brief write servo motor angle between 0 to 180 degree.
   *
   *  @return true
   */
  bool write_angle(int angle);

  /*! @brief stop Servo motor control
   *
   *  @return true
   */
  bool ReleasePWM();

  explicit AdafruitBBIOServo(std::string const& pwmPin);
  AdafruitBBIOServo();
  virtual ~AdafruitBBIOServo();

 private:
  std::string pin_key;
  Pwm pwm;
  float max_ms;
  float min_ms;
  int pwmFreq;
  uint64_t period;
};

} /* namespace bbio */
} /* adafruit */
#endif /* ADAFRUIT_BBIO_SERVO_H_ */
