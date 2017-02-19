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

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


int main(int argc, char **argv) {
	if (argc != 3)
		cout << "Usage: testbench.exe <COM> <COM>" << endl << "Analyses the precision and resolution of DMM motors and timming belt connection by sampling minimal steps." << endl;

	try {
		return test(argv[1], argv[2]);
	}
	catch (exception &e) {
		cerr << "Unhandled Exception: " << e.what() << endl;
	}
}

int test(string port1, string port2) {

	long dist = 200;

	/* Setup log file */
	cout << "Sample Datetime TargetPos Motor1Pos Motor2Pos" << endl;

	/* Set up external Gauge control */
	Gauge gauge;

	/* Connect to Serial */
	serial::Serial cSerial1(port1, 38400, serial::Timeout::simpleTimeout(1000));
	serial::Serial cSerial2(port2, 38400, serial::Timeout::simpleTimeout(1000));
	
	/*cout << "Is the serial port open? ";
	if (cSerial1.isOpen() && cSerial2.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;
	*/
	
	/* Initialize and configure motors */
	map<unsigned char, DMM::Motor*> pMotors;
	pMotors[1] = new DMM::Motor(1, &cSerial1);
	pMotors[2] = new DMM::Motor(2, &cSerial2);

	DMM::setDriveConfig(*pMotors[1], 0, 0, 0, 0);
	DMM::setDriveConfig(*pMotors[2], 0, 0, 0, 1);
	
	
	Sleep(200);

	
	/* Start looping */
	while (TRUE) {

			/* Read position */
			DMM::readPosition(*pMotors[1]);
			DMM::readPosition(*pMotors[2]);
			cSerial1.flush();
			cSerial2.flush();

			while (pMotors[1]->absPosition_readFlag || pMotors[2]->absPosition_readFlag) {// Waits for positions to return

				if (pMotors[1]->absPosition_readFlag) {
					/* Try to read motor 1 */

					if (pMotors[1]->absPosition_readFlag == 255) {
						/* Ask for position again*/
						DMM::readPosition(*pMotors[1]);
						pMotors[1]->absPosition_readFlag = 0;
					}

					pMotors[1]->absPosition_readFlag++;
					DMM::readPackages(cSerial1, pMotors);
					cSerial1.flush();
				}

				if (pMotors[2]->absPosition_readFlag) {
					/* Try to read motor 2 */

					if (pMotors[2]->absPosition_readFlag == 255) {
						/* Ask for position again*/
						DMM::readPosition(*pMotors[2]);
						pMotors[2]->absPosition_readFlag = 0;
					}

					pMotors[2]->absPosition_readFlag++;
					DMM::readPackages(cSerial2, pMotors);
					cSerial1.flush();
				}
			}

			if (abs(pMotors[2]->absPosition) > 2) {
				DMM::goRelativePos(*pMotors[1], -pMotors[2]->absPosition*2);
				//DMM::goAbsolutePos(*pMotors[2], j);
				cSerial1.flush();
			}

			//cSerial2.flush();

			//gauge.measure();

			cout << pMotors[1]->absPosition << " " << pMotors[2]->absPosition << endl;
		
	}

	
	return 0;
}