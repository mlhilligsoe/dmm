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
#include "dmm/dmm.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


int main() {
	

	// Connect to Serial

	while (true) {
		
		// Initialize motors
		

		// Instruct motor 1 to move
		DMM::goRelativePos(100);

		// Read positions of motors 1 and 2 until motor 1 is on range
		while (DMM::readPosOnRange(1)) {
			DMM::readPosition(1);
			DMM::readPosition(2);
		}
		
		// Instruct motor 1 to move back
		DMM::goRelativePos(1, -100);

		// Read positions of motors 1 and 2 until motor 1 is on range
		while (DMM::readPosOnRange(1)) {
			DMM::readPosition(1);
			DMM::readPosition(2);
		}
	}

}