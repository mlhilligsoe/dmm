
#include <string>
#include <iostream>
#include <cstdio>
#include <map>

#include "serial/serial.h"
#include "dmm/comm.h"
#include "dmm/motor.h"
#include "dmm/helper.h"

#define DEBUG false
#define INFO false

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::map;

namespace DMM {

	void sendPackage(Motor& motor, unsigned char packageLength, unsigned char B[8])
	{
		/* Send Package */
		motor.sID->write(B, packageLength);

		/* Output */
		if (DEBUG) {
			cout << pt() << " Output:" << std::hex;
			for (int i = 0; i < packageLength; i++) {
				cout << " 0x" << (int)B[i];
			}
			cout << std::dec << endl;
		}

	}

	void readPackages(serial::Serial& cSerial, map<unsigned char, Motor*> pMotors)
	{
		/* Check if data is available */
		while (cSerial.available()) {
			unsigned char motorID, packageLength;
			unsigned char B[8];

			/* Read first byte */
			cSerial.read(&B[0], 1);

			/* Continue if current byte is start byte */
			if ((B[0] >> 7) == 0) {					// 0XXXXXXX

				motorID = B[0] & 0x7f;					// 0IIIIIII

				/* Read second byte */
				cSerial.read(&B[1], 1);
				packageLength = 4 + ((B[1] >> 5) & 0x03);	// 1LLFFFFF

				/* Read remaining bytes into array */
				cSerial.read(&B[2], packageLength - 2);

				/* Continue if checksum is correct */
				if ((NotStartBit | calcCRC(packageLength, B)) == B[packageLength - 1]) {

					/* Print input */
					if (DEBUG) {
						/* Output */
						cout << pt() << " Input:" << std::hex;
						for (int i = 0; i < packageLength; i++) {
							cout << " 0x" << (int)B[i];
						}
						cout << std::dec << endl;
					}

					/* Interpret package content */
					interpretPackage(*pMotors[motorID], packageLength, B);
				}
			}
		}
	}

	void interpretPackage(Motor& motor, unsigned char packageLength, unsigned char B[8])
	{
		unsigned char packageFunction;
		packageFunction = B[1] & 0x1f;				// 1LLFFFFF

		switch (packageFunction)
		{
		case Is_Drive_ID:
			isDriveID(motor, B);
			break;
		case Is_Status:
			isStatus(motor, B);
			break;
		case Is_Config:
			isConfig(motor, B);
			break;
		case Is_AbsPos32:
			isAbsPosition(motor, packageLength, B);
			break;
		case Is_TrqCurrent:
			isTorqueCurrent(motor, packageLength, B);
			break;
		default:;
		}
	}

	unsigned char calcCRC(unsigned char packageLength, unsigned char B[8])
	{
		char CRC = 0;

		for (int i = 0; i < packageLength - 1; i++)
		{
			CRC += B[i];
		}

		return CRC;
	}

	long calcSignedValue(unsigned char packageLength, unsigned char B[8])
	{
		long lVal;

		lVal = (char)(B[2] << 1) >> 1;					// Extends the 7 data bits of the third byte to 32bits, while including sign.
		for (int i = 3; i < packageLength - 1; i++)
		{
			lVal = (lVal << 7) + (B[i] & 0x7f);			// Shift 7 bits to the left and add the 7 data bits of the next byte.
		}

		return(lVal);
	}

	void setOrigin(Motor& motor) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Set_Origin;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;											// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | 0;											// 10000000 Dummy
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> SetOrigin" << endl;

	}

	void goAbsolutePos(Motor& motor, int pos) {

		/* Define package parameters */
		unsigned char packageLength = 7;
		unsigned char functionCode = Go_Absolute_Pos;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;											// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | (unsigned char)(pos >> 21) & 0x0000007f;	// 1XXXXXXX Bit 22-28
		B[3] = NotStartBit | (unsigned char)(pos >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
		B[4] = NotStartBit | (unsigned char)(pos >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
		B[5] = NotStartBit | (unsigned char)pos & 0x0000007f;			// 1XXXXXXX Bit 1-7
		B[6] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		/* Output */
		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> goAbsolutePos=" << pos << endl;

	}

	void goRelativePos(Motor& motor, int pos) {

		/* Define package parameters */
		unsigned char packageLength = 7;
		unsigned char functionCode = Go_Relative_Pos;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;											// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | (unsigned char)(pos >> 21) & 0x0000007f;	// 1XXXXXXX Bit 22-28
		B[3] = NotStartBit | (unsigned char)(pos >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
		B[4] = NotStartBit | (unsigned char)(pos >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
		B[5] = NotStartBit | (unsigned char)pos & 0x0000007f;			// 1XXXXXXX Bit 1-7
		B[6] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		/* Output */
		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> goRelativePos=" << pos << endl;

	}

	void readDriveID(Motor& motor) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Read_Drive_ID;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;											// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | 0;											// 10000000 Dummy
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Read_DriveID" << endl;

	}

	void setDriveConfig(Motor& motor, unsigned char cmdMode, unsigned char moveMode, unsigned char servoMode, unsigned char active) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Set_Drive_Config;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;											// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | ((cmdMode << 0) + (moveMode << 2) + (servoMode << 3) + (active << 5));		// 10DCCBAA Drive Config
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO) {
			cout << pt() << " m" << (int)motor.ID << " >> Set_DriveConfig";
			cout << " cmdMode=" << (int)cmdMode << " moveMode=" << (int)moveMode << " servoMode=" << (int)servoMode << " active=" << (int)active << endl;
		}

	}

	void readDriveConfig(Motor& motor) {

		if (!motor.config_readFlag) {
			/* Define package parameters */
			unsigned char packageLength = 4;
			unsigned char functionCode = Read_Drive_Config;

			/* Define package */
			unsigned char B[8];
			B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
			B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
			B[2] = NotStartBit | 0;											// 10000000 Dummy
			B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

			/* Send packet */
			sendPackage(motor, packageLength, B);

			if (INFO)
				cout << pt() << " m" << (int)motor.ID << " >> Read_DriveConfig" << endl;
		}
	}

	void readDriveStatus(Motor& motor) {

		if (!motor.status_readFlag) {
			/* Define package parameters */
			unsigned char packageLength = 4;
			unsigned char functionCode = Read_Drive_Status;

			/* Define package */
			unsigned char B[8];
			B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
			B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
			B[2] = NotStartBit | 0;											// 10000000 Dummy
			B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

			/* Send packet */
			sendPackage(motor, packageLength, B);

			if (INFO)
				cout << pt() << " m" << (int)motor.ID << " >> Read_DriveStatus" << endl;
		}
	}

	void turnConstSpeed(Motor& motor, int speed) {

		/* Define package parameters */
		unsigned char packageLength = 6;
		unsigned char functionCode = Turn_ConstSpeed;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | (unsigned char)(speed >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
		B[3] = NotStartBit | (unsigned char)(speed >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
		B[4] = NotStartBit | (unsigned char)speed & 0x0000007f;			// 1XXXXXXX Bit 1-7
		B[5] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Turn_ConstSpeed=" << speed << endl;

	}

	void readPosition(Motor& motor) {

		//if (!motor.absPosition_readFlag) {
			/* Define package parameters */
			unsigned char packageLength = 4;
			unsigned char functionCode = General_Read;

			/* Define package */
			unsigned char B[8];
			B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
			B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
			B[2] = NotStartBit | Is_AbsPos32;								// 10011011 Request position
			B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

			/* Send packet */
			sendPackage(motor, packageLength, B);

			/* Set flag */
			motor.absPosition_readFlag = 0x01;

			/* Output */
			if (INFO)
				cout << pt() << " m" << (int)motor.ID << " >> Read_Position" << endl;
		//}
	}

	void readTorqueCurrent(Motor& motor) {

		if (!motor.torqueConst_readFlag) {
			/* Define package parameters */
			unsigned char packageLength = 4;
			unsigned char functionCode = General_Read;

			/* Define package */
			unsigned char B[8];
			B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
			B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
			B[2] = NotStartBit | Is_TrqCurrent;								// 10011110 Request Torque Current
			B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

			/* Send packet */
			sendPackage(motor, packageLength, B);

			/* Output */
			if (INFO)
				cout << pt() << " m" << (int)motor.ID << " >> Read_TorqueCurrent" << endl;
		}
	}

	void setMainGain(Motor& motor, unsigned char gain) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Set_MainGain;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | gain;										// 1XXXXXXX Gain
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Set_MainGain=" << gain << endl;

	}

	void setSpeedGain(Motor& motor, unsigned char gain) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Set_SpeedGain;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | gain;										// 1XXXXXXX Gain
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Set_SpeedGain=" << gain << endl;

	}

	void setIntGain(Motor& motor, unsigned char gain) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Set_IntGain;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | gain;										// 1XXXXXXX Gain
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Set_IntGain=" << gain << endl;

	}

	void setTorqueCons(Motor& motor, unsigned char constant) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Set_TrqCons;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | constant;									// 1XXXXXXX Gain
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Set_TorqueCons=" << constant << endl;

	}

	void setSpeedLimit(Motor& motor, unsigned char speed) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Set_HighSpeed;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | speed;										// 1XXXXXXX Speed
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Set_SpeedLimit=" << speed << endl;

	}

	void setAccelLimit(Motor& motor, unsigned char accel) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = Set_HighAccel;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | accel;										// 1XXXXXXX Accel
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Set_AccelLimit=" << accel << endl;

	}

	void setGearNumber(Motor& motor, unsigned short gearN) {

		/* Define package parameters */
		unsigned char packageLength = 5;
		unsigned char functionCode = Set_GearNumber;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & motor.ID;										// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | (unsigned char)(gearN >> 7) & 0x007f;		// 1XXXXXXX Bit 8-14
		B[3] = NotStartBit | (unsigned char)gearN & 0x007f;				// 1XXXXXXX Bit 1-7
		B[4] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(motor, packageLength, B);

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " >> Set_GearNumber=" << gearN << endl;

	}

	void isDriveID(Motor& motor, unsigned char B[8])
	{
		// ToDo

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " << Is_Drive_ID=" << (B[2] & 0x7f) << endl;
	}

	void isStatus(Motor& motor, unsigned char B[8])
	{
		motor.statusOnPosition = (B[2] & 0x7f) & 0x01;
		motor.statusServoMode = ((B[2] & 0x7f) & 0x02) >> 1;
		motor.statusAlarm = ((B[2] & 0x7f) & 0x1c) >> 2;
		motor.statusMotion = ((B[2] & 0x7f) & 20) >> 5;
		motor.statusPin2JP3 = ((B[2] & 0x7f) & 40) >> 6;
		motor.status_readFlag = 0x00;

		/* Output */
		if (INFO) {
			cout << pt() << " m" << (int)motor.ID << " << Is_Status=" << (B[2] & 0x7f);
			cout << " OnPosition=" << (int)motor.statusOnPosition << " ServoMode=" << (int)motor.statusServoMode << " Alarm=" << (int)motor.statusAlarm << " Motion=" << (int)motor.statusMotion << " Pin2JP3=" << (int)motor.statusPin2JP3 << endl;
		}

	}

	void isConfig(Motor& motor, unsigned char B[8])
	{
		// ToDo

		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " << Is_Config=" << (B[2] & 0x7f);
	}

	void isAbsPosition(Motor& motor, unsigned char packageLength, unsigned char B[8])
	{
		if (!motor.absPosition_readFlag == 0x00)
			++motor.count;
		
		motor.absPosition = calcSignedValue(packageLength, B);
		motor.absPosition_readFlag = 0x00;
		
		/* Output */
		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " << Is_AbsPos32=" << motor.absPosition << endl;

	}

	void isTorqueCurrent(Motor& motor, unsigned char packageLength, unsigned char B[8])
	{
		motor.torqueCurrent = calcSignedValue(packageLength, B);
		motor.torqueCurrent_readFlag = 0x00;

		/* Output */
		if (INFO)
			cout << pt() << " m" << (int)motor.ID << " << Is_AbsPos32=" << motor.torqueConst << endl;

	}

}