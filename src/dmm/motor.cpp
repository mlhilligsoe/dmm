
#include <string>
#include <iostream>
#include <cstdio>
#include <map>

#include "serial/serial.h"
#include "dmm/motor.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::map;

using namespace DMM;

Motor::Motor(unsigned char id, serial::Serial* sid) {
	
	/* Motor parameters */
	mainGain = speedGain = intGain = torqueConst = speedLimit = accelLimit = posOnRange = 0;
	gearN = 0;
	statusOnPosition = statusServoMode = statusAlarm = statusMotion = statusPin2JP3 = 0;
	cmdMode = moveMode = servoMode = active = 0;
	absPosition = torqueCurrent = 0;

	/* Read flags */
	ID_readFlag = posOnRange_readFlag = 0;
	mainGain_readFlag = speedGain_readFlag = intGain_readFlag = torqueConst_readFlag = speedLimit_readFlag = accelLimit_readFlag = gearN_readFlag = 0;
	status_readFlag = 0;
	config_readFlag = 0;
	absPosition_readFlag = torqueCurrent_readFlag = 0;
	

	ID = id;
	sID = sid;

}

Motor::Motor()
{
	/* Motor parameters */
	ID = 127;
	mainGain = speedGain = intGain = torqueConst = speedLimit = accelLimit = posOnRange = 0;
	gearN = 0;
	statusOnPosition = statusServoMode = statusAlarm = statusMotion = statusPin2JP3 = 0;
	cmdMode = moveMode = servoMode = active = 0;
	absPosition = torqueCurrent = 0;

	/* Read flags */
	ID_readFlag = posOnRange_readFlag = 0;
	mainGain_readFlag = speedGain_readFlag = intGain_readFlag = torqueConst_readFlag = speedLimit_readFlag = accelLimit_readFlag = gearN_readFlag = 0;
	status_readFlag = 0;
	config_readFlag = 0;
	absPosition_readFlag = torqueCurrent_readFlag = 0;

	sID = NULL;

}

Motor::~Motor()
{
}
