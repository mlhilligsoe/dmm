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
#include "testbench/testbench.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;




int main(int argc, char **argv) {
	if (argc != 3)
		cout << "Usage: testbench.exe <COM> <COM>" << endl << "Description" << endl;

	try {
		return test(argv[1], argv[2]);
	}
	catch (exception &e) {
		cerr << "Unhandled Exception: " << e.what() << endl;
	}
}



int test(string port1, string port2) {

	INPUT ip;
	ip.type = INPUT_KEYBOARD;
	ip.ki.wScan = 0; // hardware scan code for key
	ip.ki.time = 0;
	ip.ki.dwExtraInfo = 0;
	ip.ki.wVk = VK_F7; // virtual-key code for the "F7" key

	long m = 30000;

	// Connect to Serial
	serial::Serial cSerial1(port1, 38400, serial::Timeout::simpleTimeout(1000));
	serial::Serial cSerial2(port2, 38400, serial::Timeout::simpleTimeout(1000));
	
	cout << "Is the serial port open? ";
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

		// Press the "F7" key
		ip.ki.dwFlags = 0; // 0 for key press
		SendInput(1, &ip, sizeof(INPUT));

		// Release the "F7" key
		ip.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
		SendInput(1, &ip, sizeof(INPUT));

		Sleep(1000);

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