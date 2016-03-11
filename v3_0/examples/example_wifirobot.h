/*  This is a example of creating a wifi control robot based on Beablebone
*  by using the BlackLib of my own version
*  https://www.youtube.com/embed/v77tkjFmZqY
*
*  Created on: Nov 14, 2015
*  Author: Chao Li
*  Email:forward.li.kth@gmail.com
*  Blog: http://forwardkth.github.io/
*/
#ifndef BLACKLIB_EXAMPLES_EXAMPLE_WIFIROBOT_H_
#define BLACKLIB_EXAMPLES_EXAMPLE_WIFIROBOT_H_

#include <string>
#include <iostream>
#include <vector>
#include <cstdlib>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include "../BlackThread/BlackThread.h"
#include "../BlackMutex/BlackMutex.h"
#include "../BlackServo/BlackServo.h"
#include "../BlackGPIO/BlackGPIO.h"
#include "../BlackUART/BlackUART.h"

#define TCP_ADDR "192.168.1.79"
#define TCP_PORT_RX 2002
#define TCP_PORT_TX 2003

int laser_status = 0;
int servoxy_angle = 82;
int servoz_angle = 10;
int ultra_distance = 100;
int lowlen = 0;
int highlen = 0;

using namespace BlackLib;
// TCP thread for order receiver and execution
class TCP_receiver : public BlackThread {
public:
	TCP_receiver(
		BlackServo &XY,
		BlackServo &Z,
		int &laser_status,
		int &servoxy_angle,
		int &servoz_angle,
		BlackGPIO &io1_12,
		BlackGPIO &io1_13,
		BlackGPIO &io1_14,
		BlackGPIO &io1_15,
		BlackGPIO &io1_6)
		:xy(XY), z(Z),
		laser(laser_status),
		xyangle(servoxy_angle),
		zangle(servoz_angle),
		gpio1_12(io1_12),
		gpio1_13(io1_13),
		gpio1_14(io1_14),
		gpio1_15(io1_15),
		gpio1_6(io1_6) {
	}
	void onStartHandler() {
		//TCP init
		int serverSocket;
		struct sockaddr_in serverAddr;
		struct sockaddr_in clientAddr;

		int port = TCP_PORT_RX;   // TCP server port

							   // creat and initialize a socket
		serverSocket = socket(AF_INET, SOCK_STREAM, 0);

		// optional setting，try to avoid the server can not be restart quickly
		int val = 1;
		setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

		// define the listening port and address
		serverAddr.sin_family = AF_INET;
		serverAddr.sin_port = htons(port);
		serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		memset(&(serverAddr.sin_zero), 0, 8);
		int rc = bind(serverSocket, (struct sockaddr*) &serverAddr,
			sizeof(struct sockaddr));
		if (rc == -1) {
			std::cout << "bind failed!" << std::endl;
			exit(1);
		}

		// start listening and the max client number is 5
		rc = listen(serverSocket, 100);
		if (rc == -1) {
			std::cout << "listen failed!" << std::endl;
			exit(1);
		}

		// waiting for a connection
		int sock;
		int clientAddrSize = sizeof(struct sockaddr_in);
		std::cout << "TCP RX socket created!" << std::endl;
		//std::cout << "read!"<<"  "<<order<< std::endl;
		//std::cout << "zangle!"<<"  "<<zangle<< std::endl;
		//std::cout << "xyangle!"<<"  "<<xyangle<< std::endl;
		

		//TCP_receive thread loop
		while (1) {
			sock = accept(serverSocket,
				(struct sockaddr*) &clientAddr,
				(socklen_t*)&clientAddrSize);

			char order;
			recv(sock, &order, 1, 0);

			if (order) {
				if (order == '5') { //PLZ UP
					if (zangle <= 78) {
						zangle += 2;
						z.write_angle(zangle);
						this->msleep(500);
						z.ReleasePWM();
					}
					else if (zangle > 80) {
						zangle = 80;
					}
				}
				else if (order == '6') { //PLZ DOWN
					if (zangle >= 12) {
						zangle -= 2;
						z.write_angle(zangle);
						this->msleep(500);
						z.ReleasePWM();
					}
					else if (zangle < 10) {
						zangle = 10;
					}
				}
				else if (order == '7') { //PLZ LEFT
					if (xyangle >= 42) {
						xyangle -= 2;
						xy.write_angle(xyangle);
						this->msleep(500);
						xy.ReleasePWM();
					}
					else if (xyangle < 40) {
						xyangle = 40;
					}
				}
				else if (order == '8') { //PLZ RIGHT
					if (xyangle <= 132) {
						xyangle += 2;
						xy.write_angle(xyangle);
						this->msleep(500);
						xy.ReleasePWM();
					}
					else if (xyangle > 134) {
						xyangle = 134;
					}
				}
				else if (order == '9') { //RESET PLZ
					xyangle = 82;
					zangle = 10;
					z.write_angle(10);
					xy.write_angle(82);
					this->msleep(500);
				}
				else if (order == '0') { //motor stop
					gpio1_13.setValue(low);
					gpio1_12.setValue(low);
					gpio1_14.setValue(low);
					gpio1_15.setValue(low);
					this->msleep(10);
				}
				else if (order == '1') { // motor forward
					if (ultra_distance >= 200) {
						gpio1_12.setValue(high);
						gpio1_15.setValue(high);
					}
					this->msleep(10);
				}
				else if (order == '2') { // motor backward
					gpio1_14.setValue(high);
					gpio1_13.setValue(high);
					this->msleep(10);
				}
				else if (order == '3') { // motor turn left
					gpio1_14.setValue(high);
					gpio1_12.setValue(high);
					this->msleep(10);
				}
				else if (order == '4') { // motor turn right
					gpio1_13.setValue(high);
					gpio1_15.setValue(high);
					this->msleep(10);
				}
				else if (order == 'l') { // laser on/off
					if (0 == laser) {
						gpio1_6.setValue(high);
						laser = 1;
					}
					else {
						gpio1_6.setValue(low);
						laser = 0;
					}
					//this->sleep(1);
				}
			}
			else {
				std::cout << "no order received!" << order << std::endl;
				gpio1_13.setValue(low);
				gpio1_12.setValue(low);
				gpio1_14.setValue(low);
				gpio1_15.setValue(low);
				gpio1_6.setValue(low);
			}
		}

		close(sock);
		return;
	}

private:
	BlackServo &xy;
	BlackServo &z;
	int &laser;
	int &xyangle;
	int &zangle;
	BlackGPIO gpio1_12;
	BlackGPIO gpio1_13;
	BlackGPIO gpio1_14;
	BlackGPIO gpio1_15;
	BlackGPIO gpio1_6;
};

// TCP thread for cmd receiving and execution
class TCP_sender : public BlackLib::BlackThread {
public:
	TCP_sender() {};

	void onStartHandler() {
		//TCP init
		int serverSocket;
		struct sockaddr_in serverAddr;
		struct sockaddr_in clientAddr;

		int port = TCP_PORT_TX;   // TCP server TX port

								  // creat and initialize a socket
		serverSocket = socket(AF_INET, SOCK_STREAM, 0);

		// optional setting，try to avoid the server can not be restart quickly
		int val = 1;
		setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

		// define the listening port and address
		serverAddr.sin_family = AF_INET;
		serverAddr.sin_port = htons(port);
		serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		memset(&(serverAddr.sin_zero), 0, 8);
		int rc = bind(serverSocket, (struct sockaddr*) &serverAddr,
			sizeof(struct sockaddr));
		if (rc == -1) {
			std::cout << "bind failed!" << std::endl;
			exit(1);
		}

		// start listening and the max client number is 5
		rc = listen(serverSocket, 100);
		if (rc == -1) {
			std::cout << "listen failed!" << std::endl;
			exit(1);
		}

		// waiting for a connection
		int sock;
		int clientAddrSize = sizeof(struct sockaddr_in);
		std::cout << "TCP TX socket created!" << std::endl;
	}

};

// Ultra sound thread for distance detection
class Ultrasound : public BlackLib::BlackThread {
public:
	Ultrasound(BlackUART &serial,
		int &distance, int &low, int &high)
		: uart(serial), range(distance), Lowlen(low), Highlen(high) {

	}

	void onStartHandler() {
		uart.setReadBufferSize(16);
		if (uart.open(BlackLib::ReadWrite)) {
			std::cout << std::endl;
			std::cout << "Device Path     : " << uart.getPortName() << std::endl;
			std::cout << "Read Buf. Size  : " << uart.getReadBufferSize() << std::endl;
			std::cout << "BaudRate In/Out : " << uart.getBaudRate(BlackLib::input) << "/"
				<< uart.getBaudRate(BlackLib::output) << std::endl;
			std::cout << "Character Size  : " << uart.getCharacterSize() << std::endl;
			std::cout << "Stop Bit Size   : " << uart.getStopBits() << std::endl;
			std::cout << "Parity          : " << uart.getParity() << std::endl << std::endl;

			char writeArr[4] = "U\r\n";
			char readArr[22];
			memset(readArr, 0, sizeof(readArr));
			string readt;
			while (1) {
				uart.flush(BlackLib::direction::bothDirection);
				uart.write(writeArr, sizeof(writeArr));
				msleep(500);
				uart.read(readArr, sizeof(readArr));
				Highlen = (int)readArr[0];
				Lowlen = (int)readArr[1];
				range = Highlen * 256 + Lowlen;
				//std::cout<< "distance forward: "<<range<<std::endl;
			}
			return;
		}
	}

private:
	BlackUART &uart;
	int &range;
	int &Lowlen;
	int &Highlen;
};

// wifirobot APP loop will be called in main
int wifirobot() {	
	//robot PLZ init
	BlackServo servoXY(BlackLib::EHRPWM1B);
	BlackServo servoZ(BlackLib::EHRPWM2B);
	servoXY.write_angle(servoxy_angle);
	servoZ.write_angle(servoz_angle);
	BlackLib::BlackThread::sleep(1);
	servoXY.ReleasePWM();
	servoZ.ReleasePWM();
	// robot motor init
	BlackLib::BlackGPIO GPIO1_12
		(BlackLib::GPIO_44, BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO GPIO1_13
		(BlackLib::GPIO_45, BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO GPIO1_14
		(BlackLib::GPIO_46, BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO GPIO1_15
		(BlackLib::GPIO_47, BlackLib::output, BlackLib::SecureMode);
	// robot laser init
	BlackLib::BlackGPIO GPIO1_6
		(BlackLib::GPIO_38, BlackLib::output, BlackLib::SecureMode);
	//robot ultra sound init
	BlackLib::BlackUART Usound_serial(BlackLib::UART2,
		BlackLib::Baud9600,
		BlackLib::ParityNo, //this setting is very important
		BlackLib::StopOne,
		BlackLib::Char8);
	
	//start ultrasound thread
	Ultrasound *ultras = new Ultrasound(Usound_serial, ultra_distance, lowlen, highlen);
	ultras->run();

	//start TCP_RX thread
	TCP_receiver *rev = new TCP_receiver(servoXY, servoZ,
		laser_status,
		servoxy_angle,
		servoz_angle,
		GPIO1_12,
		GPIO1_13,
		GPIO1_14,
		GPIO1_15,
		GPIO1_6);
	rev->run();
	
	//start TCP_TX thread
	//..................

	//main loop
	while(1) {

		usleep(100);
		if (ultras->isFinished()) {
			delete ultras;
			ultras = nullptr;
			break;
		} else if (rev->isFinished()) {
			delete rev;
			rev = nullptr;
			break;
		}
	}
	return 0;
}

#endif /* BLACKLIB_EXAMPLES_EXAMPLE_WIFIROBOT_H_ */
