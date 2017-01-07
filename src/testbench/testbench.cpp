
#define Go_Absolute_Pos 0x01
#define Is_AbsPos32 0x1b
#define General_Read 0x0e
#define Is_TrqCurrent 0x1e
#define Read_MainGain 0x18
#define Is_MainGain 0x10

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include "dmm/comm.h"
#include "dmm/motor.h"
#include "..\..\include\testbench\testbench.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;




int main(int argc, char **argv) {
	try {
		return test(argv[1], argv[2]);
	}
	catch (exception &e) {
		cerr << "Unhandled Exception: " << e.what() << endl;
	}
}



int test(string port1, string port2) {

	long m = 30000;

	// Connect to Serial
	serial::Serial cSerial1(port1, 38400, serial::Timeout::simpleTimeout(1000));
	serial::Serial cSerial2(port2, 38400, serial::Timeout::simpleTimeout(1000));
	
	cout << "Is the serial port open?";
	if (cSerial1.isOpen() && cSerial1.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;

	cout << "Testing started" << endl;

	map<unsigned char, DMM::Motor*> pMotors;
	pMotors[1] = new DMM::Motor(1, &cSerial1);
	pMotors[2] = new DMM::Motor(2, &cSerial2);

	DMM::setDriveConfig(*pMotors[2], 0, 0, 0, 1);
	
	while (true) {

		// Read motor 2 position
		DMM::readPosition(*pMotors[2]);
		DMM::readPackages(cSerial2, pMotors);
		
		// Set motor 1 position
		if(!pMotors[1]->absPosition_readFlag)
			DMM::goAbsolutePos(*pMotors[1], pMotors[2]->absPosition);

		

		/*
		DMM::goAbsolutePos(*pMotors[1], m);
		
		// Read positions of motors 1 and 2 until motor 1 is on range
		while (pMotors[1]->absPosition_readFlag || abs(pMotors[1]->absPosition - m) > 3) {
			DMM::readPosition(*pMotors[1]);
			DMM::readPackages(cSerial1, pMotors);
		}


		cout << endl << endl;
		
		DMM::goAbsolutePos(*pMotors[1], -m);
		
		// Read positions of motors 1 and 2 until motor 1 is on range
		while (pMotors[1]->absPosition_readFlag || abs(pMotors[1]->absPosition - (-m)) > 3) {
			DMM::readPosition(*pMotors[1]);
			DMM::readPackages(cSerial1, pMotors);
		}
		*/
	}

	return 0;
}