/*
 *
 * BlackLib Library controls Beaglebone's Servo motor by PWM.
 *
 *  Created on: Oct 25, 2015
 *      Author: Chao Li
 *      Email:forward.li.kth@gmail.com
 *      Blog: http://forwardkth.github.io/
 */


#ifndef EXAMPLE_SERVO_H_
#define EXAMPLE_SERVO_H_

#include <string>
#include <iostream>
#include "../BlackServo/BlackServo.h"

void example_Servo() {
  std::cout << "Servo control function test begin" << std::endl;

  BlackLib::BlackServo servoXY(BlackLib::EHRPWM1B);
  BlackLib::BlackServo servoZ(BlackLib::EHRPWM2B);

  int i = 2;

  while(i) {
    for(int angle1 = 40; angle1 != 136; ++angle1) {
	  servoXY.write_angle(angle1);
	  usleep(100000);
    }
	for(int angle2 = 135;angle2 != 39; --angle2) {
	  servoXY.write_angle(angle2);
	  usleep(100000);
    }
    for(int angle3 = 4; angle3 != 81; ++angle3) {
	  servoZ.write_angle(angle3);
	  usleep(100000);
	}
	for(int angle4 = 80; angle4 != 3; --angle4) {
	  servoZ.write_angle(angle4);
	  usleep(100000);
	}
	--i;
  }
  servoXY.ReleasePWM();
  servoZ.ReleasePWM();
}
#endif /* EXAMPLE_SERVO_H_ */
