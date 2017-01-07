#ifndef DMM_MOTOR_H
#define DMM_MOTOR_H

namespace DMM {
	class Motor {
	public:
		/* Motor parameters */
		unsigned char ID = 127;
		unsigned char mainGain, speedGain, intGain, torqueConst, speedLimit, accelLimit, posOnRange = 0;
		unsigned short gearN = 0;
		unsigned char statusOnRange, statusServoMode, statusAlarm, statusMotion, statusPin2JP3 = 0;
		unsigned char cmdMode, moveMode, servoMode, active = 0;
		int absPosition, torqueCurrent = 0;

		/* Read flags */
		unsigned char ID_readFlag, posOnRange_readFlag = 0;
		unsigned char mainGain_readFlag, speedGain_readFlag, intGain_readFlag, torqueConst_readFlag, speedLimit_readFlag, accelLimit_readFlag, gearN_readFlag = 0;
		unsigned char status_readFlag = 0;
		unsigned char config_readFlag = 0;
		unsigned char absPosition_readFlag, torqueCurrent_readFlag = 0;
		
		unsigned char sID;

		/* Constructor */
		Motor(const unsigned char id, const unsigned char sid);
		Motor();
		~Motor();


	};

#endif

