#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep and keystroke
#ifdef _WIN32
#define WINVER 0x0500
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include "dmm/comm.h"
#include "dmm/motor.h"
#include "dmm\helper.h"
#include "testbench/testbench.h"
#include "testbench\gauge.h"
#include <chrono>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::chrono::system_clock;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::time_point;




int main(int argc, char **argv) {
	if (argc != 3)
		cout << "Usage: testbench.exe <COM> <COM>" << endl << "Analyses the communication frequency of DMM motors." << endl;

	try {
		return test(argv[1], argv[2]);
	}
	catch (exception &e) {
		cerr << "Unhandled Exception: " << e.what() << endl;
	}
}



int test(string port1, string port2) {

	
	/* Set up external Guage control */
	Gauge gauge;

	/* Connect to Serial */
	serial::Serial cSerial1(port1, 38400, serial::Timeout::simpleTimeout(1000));
	serial::Serial cSerial2(port2, 38400, serial::Timeout::simpleTimeout(1000));

	cout << "Is the serial port open? ";
	if (cSerial1.isOpen() && cSerial1.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;


	/* Initialize and configure motors */
	map<unsigned char, DMM::Motor*> pMotors;
	pMotors[1] = new DMM::Motor(1, &cSerial1);
	pMotors[2] = new DMM::Motor(2, &cSerial2);
	pMotors[3] = new DMM::Motor(3, &cSerial1);
	pMotors[4] = new DMM::Motor(4, &cSerial2);

	DMM::setDriveConfig(*pMotors[2], 0, 0, 0, 1);

	/* Set start positions */
	DMM::setOrigin(*pMotors[1]);
	DMM::setOrigin(*pMotors[2]);
	
	Sleep(200);

	/* Do inital sample */
	DMM::readPosition(*pMotors[1]);
	DMM::readPosition(*pMotors[2]);
	

	while (pMotors[1]->absPosition_readFlag || pMotors[2]->absPosition_readFlag) { // Waits for positions to return
		DMM::readPackages(cSerial1, pMotors);
		DMM::readPackages(cSerial2, pMotors);
	}

	
	time_point<system_clock> start_time, last_msg_time, current_time;
	int i;

	start_time = system_clock::now();
	last_msg_time = system_clock::now();
	current_time = system_clock::now();
	i = 0;
	
	

	/* Start looping */
	while (current_time < start_time + seconds(10)) {
		
		if (current_time > last_msg_time + milliseconds(50)) {
			DMM::goAbsolutePos(*pMotors[1], i);
			DMM::readPosition(*pMotors[1]);
			last_msg_time = current_time;
		}
		cSerial1.flush();
		DMM::readPackages(cSerial1, pMotors);
		cSerial1.flush();
		++i;
		current_time = system_clock::now();
		

		/*		
		DMM::readPosition(*pMotors[1]);
		DMM::readPosition(*pMotors[2]);
		DMM::readPosition(*pMotors[3]);
		DMM::readPosition(*pMotors[4]);
		
		DMM::goAbsolutePos(*pMotors[1], i * 1000);
		DMM::goAbsolutePos(*pMotors[2], i * 1000);
		DMM::goAbsolutePos(*pMotors[3], i * 1000);
		DMM::goAbsolutePos(*pMotors[4], i * 1000);

		while (pMotors[1]->absPosition_readFlag && pMotors[2]->absPosition_readFlag) {// Waits for positions to return
			DMM::readPackages(cSerial1, pMotors);
		}
		++i;
		//cout << i << endl;
		*/
	}
	
	cout << "Communication cycles in 1 seconds: " << pMotors[1]->count << endl;
	cout << "Loops: " << i << endl;
	
	

	return 0;
}