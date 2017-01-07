

#include <vector>
#include <string>

#include "serial\serial.h"
#include "dmm\comm.h"

using std::vector;
using std::string;

vector<serial::Serial*> pSerials;

void addSerialConnection(string port) {
	
	serial::Serial * tSerial = new serial::Serial(port, 38400, serial::Timeout::simpleTimeout(1000));

	if (tSerial->open) {
		pSerials.push_back(tSerial);
		discoverMotors(pSerials.size());
	}
	else {
		cerr << "Could not open port:" << port << endl;
	}

}

void discoverMotors(int sId) {
	DMM::readDriveID(127);

	while (pSerials[sId]->waitReadable()) {
		DMM::readPackages(*pSerials[sId]);
	}
	
}