
#include <map>
#include <vector>
#include <string>
#include <cstdio>
#include <iostream>

#include "serial\serial.h"
#include "dmm\comm.h"
#include "dmm\motor.h"
#include "dmm\controller.h"

using std::vector;
using std::map;
using std::string;
using std::cout;
using std::cerr;
using std::endl;

namespace DMM {

	map<unsigned char, Motor*> pMotors;
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

	void addSerialConnection(string port) {

		void discoverMotors(int sId) {
			DMM::readDriveID(127);

			while (pSerials[sId]->waitReadable()) {
				DMM::readPackages(*pSerials[sId]);
			}

		}
	}
	



}