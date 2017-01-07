#ifndef DMM_CONTROLLER_H
#define DMM_CONTROLLER_H

#include <map>
#include <string>

#include "serial\serial.h"
#include "dmm\motor.h"

using std::map;
using std::string;

namespace DMM {
	void addSerialConnection(string port);
}

#endif