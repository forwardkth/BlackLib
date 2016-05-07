/*
 * BlackServo.cpp
 *
 * BlackLib Library controls Beaglebone's Servo motor by PWM.
 *
 *  Created on: Oct 25, 2015
 *      Author: Chao Li
 *      Email:forward.li.kth@gmail.com
 *      Blog: http://forwardkth.github.io/
 */

#ifndef BLACKSERVO_H_
#define BLACKSERVO_H_

#include "../BlackPWM/BlackPWM.h"

namespace BlackLib {

class BlackServo: virtual private BlackPWM {
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

  BlackServo(pwmName pwmPin);
  virtual ~BlackServo();

 private:
  float max_ms;
  float min_ms;
  int pwmFreq;
  uint64_t period;
};

} /* namespace BlackLib */
#endif /* BLACKSERVO_H_ */
