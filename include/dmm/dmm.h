#ifndef DMM_H
#define DMM_H

#include "serial\serial.h"

namespace DMM {
	class Motor {
	public:
		/* Motor parameters */
		unsigned char ID = id;
		unsigned char mainGain, speedGain, intGain, torqueConst, speedLimit, accelLimit, posOnRange;
		unsigned short gearN;
		unsigned char statusOnRange, statusServoMode, statusAlarm, statusMotion, statusPin2JP3;
		unsigned char cmdMode, moveMode, servoMode, active;
		int absPosition, torqueCurrent;

		/* Read flags */
		unsigned char ID_readFlag, posOnRange_readFlag;
		unsigned char mainGain_readFlag, speedGain_readFlag, intGain_readFlag, torqueConst_readFlag, speedLimit_readFlag, accelLimit_readFlag, gearN_readFlag;
		unsigned char status_readFlag;
		unsigned char config_readFlag;
		unsigned char absPosition_readFlag, torqueCurrent_readFlag;

		/* Constructor */
		Motor(const unsigned char id);

	};

	void sendPackage(unsigned char packageLength, unsigned char B[8]);
	void readPackages();
	void interpretPackage(unsigned char packageID, unsigned char packageLength, unsigned char B[8]);
	unsigned char calcCRC(unsigned char packageLength, unsigned char B[8]);
	long calcSignedValue(unsigned char packageLength, unsigned char B[8]);
	
	
	/* Drive input functions */
	void setOrigin(unsigned char ID);
	void goAbsolutePos(unsigned char ID, int pos);
	void makeLinearLine(unsigned char ID, int line);
	void goRelativePos(unsigned char ID, int pos);
	void makeCircularArc(unsigned char ID, int pos);
	void assignDriveID(unsigned char ID, unsigned char id);
	void readDriveID(unsigned char ID);
	void setDriveConfig(unsigned char ID, unsigned char cmdMode, unsigned char moveMode, unsigned char servoMode, unsigned char active);
	void readDriveConfig(unsigned char ID);
	void readDriveStatus(unsigned char ID);
	void turnConstSpeed(unsigned char ID, int speed);
	void squareWave(unsigned char ID, int amplitude);
	void sinWave(unsigned char ID, int amplitude);
	void setFrequency(unsigned char ID, int frequency);
	void readPosition(unsigned char ID);
	void readTorqueCurrent(unsigned char ID);
	void setMainGain(unsigned char ID, unsigned char gain);
	void setSpeedGain(unsigned char ID, unsigned char gain);
	void setIntGain(unsigned char ID, unsigned char gain);
	void setTorqueCons(unsigned char ID, unsigned char constant);
	void setSpeedLimit(unsigned char ID, unsigned char speed);
	void SetAccelLimit(unsigned char ID, unsigned char accel);
	void setPosOnRange(unsigned char ID, unsigned char range);
	void setGearNumber(unsigned char ID, unsigned short gearN);
	void readMainGain(unsigned char ID);
	void readSpeedGain(unsigned char ID);
	void readIntGain(unsigned char ID);
	void readTorqueCons(unsigned char ID);
	void readSpeedLimit(unsigned char ID);
	void readAccelLimit(unsigned char ID);
	void readPosOnRange(unsigned char ID);
	void readGearNumber(unsigned char ID);

	/* Drive output functions */
	void isMainGain();
	void isSpeedGain();
	void isIntGain();
	void isTorqueCons();
	void isSpeedLimit();
	void isAccelLimit();
	void isDriveID();
	void isPosOnRange();
	void isGearNumber();
	void isStatus();
	void isConfig();
	void isAbsPosition();
	void isTorqueCurrent();
}

#endif