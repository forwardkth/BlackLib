/*
 * BlackServo.cpp
 *
 * BlackLib Library controls Beaglebone's Servo motor by PWM.
 *
 *  Created on: Oct 25, 2015
 *  Author: Chao Li
 *  Email:forward.li.kth@gmail.com
 *  Blog: http://forwardkth.github.io/
 */

#include "BlackServo.h"

namespace BlackLib {

BlackServo::BlackServo(pwmName pwmPin)
    : BlackPWM(pwmPin),
      max_ms(2.4),
      min_ms(0.5),
      pwmFreq(50),
      period(1000.0 / pwmFreq) {
  this->setDutyPercent(0.0);
  this->setPeriodTime(20, milisecond);
}

BlackServo::~BlackServo() {
  // TODO Auto-generated destructor stub
}

bool BlackServo::write_angle(int angle) {
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;
  int64_t value = ((max_ms - min_ms) / 180.0 * angle + min_ms) * 1000;
  this->setPeriodTime(period, milisecond);
  this->setLoadRatioTime(value, microsecond);
  //For debug purpose
  //std::cout << "DUTY after setting load time: \t\t" << this->getDutyValue() << std::endl;
  //std::cout << "DUTY after setting space time: \t\t" << this->getDutyValue() << std::endl;
  //std::cout << "PERIOD after setting period time: \t" << this->getPeriodValue() << std::endl;
  //std::cout <<value<<std::endl;
  return true;
}

bool BlackServo::ReleasePWM() {
  this->setDutyPercent(0.0);
  return true;
}

} /* namespace BlackLib */
