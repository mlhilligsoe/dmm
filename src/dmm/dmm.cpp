
#define StartBit 0x7f
#define NotStartBit 0x80

/* Drive function codes */
#define Set_Origin 0x00
#define Go_Absolute_Pos 0x01
#define Make_LinearLine 0x02
#define Go_Relative_Pos 0x03
#define Make_CircularArc 0x04
#define Assign_Drive_ID 0x05
#define Read_Drive_ID 0x06
#define Set_Drive_Config 0x07
#define Read_Drive_Config 0x08
#define Read_Drive_Status 0x09
#define Turn_ConstSpeed 0x0a
#define Square_Wave 0x0b
#define Sin_Wave 0x0c
#define SS_Frequency 0x0d
#define General_Read 0x0e
#define ForMotorDefine 0x0f
#define Set_MainGain 0x10
#define Set_SpeedGain 0x11
#define Set_IntGain 0x12
#define Set_TrqCons 0x13
#define Set_HighSpeed 0x14
#define Set_HighAccel 0x15
#define Set_Pos_OnRange 0x16
#define Set_GearNumber 0x17
#define Read_MainGain 0x18
#define Read_SpeedGain 0x19
#define Read_IntGain 0x1a
#define Read_TrqCons 0x1b
#define Read_HighSpeed 0x1c
#define Read_HighAccel 0x1d
#define Read_Pos_OnRange 0x1e
#define Read_GearNumber 0x1f

/* Motor function codes */
#define Is_MainGain 0x10
#define Is_SpeedGain 0x11
#define Is_IntGain 0x12
#define Is_TrqCons 0x13
#define Is_HighSpeed 0x14
#define Is_HighAccel 0x15
#define Is_Drive_ID 0x16
#define Is_PosOn_Range 0x17
#define Is_GearNumber 0x18
#define Is_Status 0x19
#define Is_Config 0x1a
#define Is_AbsPos32 0x1b
#define Is_TrqCurrent 0x1e


#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <chrono>
#include <map>
#include <Windows.h>

#include "serial/serial.h"
#include "dmm/dmm.h"

using std::string;
using std::exception;
using std::cout; 
using std::cerr;
using std::endl;
using std::vector;
using std::map;


/* Serial Communication */
//serial::Serial cSerial;


map<unsigned char, DMM::Motor> motors;
map<unsigned char, serial::Serial*> serialPts;

//DMM::Motor* motors = new DMM::Motor[3];// = { DMM::Motor(0), DMM::Motor(1), DMM::Motor(2) };


DMM::Motor::Motor(unsigned char id, const unsigned char sid) {
	
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
	
	


	ID = id;
	sID = sid;

}

DMM::Motor::Motor()
{

}

DMM::Motor::~Motor()
{
}

void DMM::sendPackage(unsigned char packageLength, unsigned char B[8]) {

}

void DMM::sendPackage(unsigned char packageLength, unsigned char B[8], serial::Serial &cSerial)
{
	cSerial.write(B, packageLength);
	
	/* Output 
	pt();
	cout << "Output: ";
	for (int i = 0; i < packageLength; i++) {
		cout << "0x" << std::hex << (int)B[i] << std::dec << " ";
	}
	cout << endl;
	*/
}

void DMM::readPackages(serial::Serial& cSerial)
{
	/* Check if data is available */
	while (cSerial.available()) {
		unsigned char packageID, packageLength;
		unsigned char B[8];
		
		/* Read first byte */
		B[0] = cSerial.read(1).at(0);

		/* Continue if current byte is start byte */
		if ( (B[0] & !StartBit) == 0) {					// 0XXXXXXX
			
			packageID = B[0] & 0x7f;					// 0IIIIIII

			/* Read second byte */
			B[1] = cSerial.read(1).at(0);
			packageLength = 4 + ((B[1] >> 5) & 0x03);	// 1LLFFFFF
			
			/* Read reamining bytes */
			cSerial.read(&B[2], packageLength - 2);

			/* Continue if checksum is correct */
			if ( (NotStartBit | calcCRC(packageLength, B)) == B[packageLength - 1]) {
				
				/* Output 
				pt();
				cout << "Input " << (int)packageLength << " bytes from " << (int)packageID << ": ";
				for (int i = 0; i < packageLength; i++) {
					cout << "0x" << std::hex << (int)B[i] << std::dec << " ";
				}
				cout << endl;
				*/

				interpretPackage(packageID, packageLength, B);
			}
		}
	}
}

void DMM::interpretPackage(unsigned char ID, unsigned char packageLength, unsigned char B[8])
{
	unsigned char packageFunction;
	packageFunction = B[1] & 0x1f;				// 1LLFFFFF

	/*
	pt();
	cout << "Function " << (int)packageFunction << ": " << endl;
	*/

	switch (packageFunction)
	{
	case Is_MainGain:
		//motors[ID].mainGain = B[2] & 0x7f;
		//motors[ID].mainGain_readFlag = 0x00;
		pt();
		cout << 'm' << (int)ID << " Is_MainGain=" << (B[2] & 0x7f) << endl;
		break;
	case Is_SpeedGain:
		//motors[ID].speedGain = B[2] & 0x7f;
		//motors[ID].speedGain_readFlag = 0x00;
		pt();
		cout << 'm' << (int)ID << " Is_SpeedGain=" << (B[2] & 0x7f) << endl;
		break;
	case Is_IntGain:
		//motors[ID].intGain = B[2] & 0x7f;
		//motors[ID].intGain_readFlag = 0x00;
		pt();
		cout << 'm' << (int)ID << " Is_IntGain=" << (B[2] & 0x7f) << endl;
		break;
	case Is_TrqCons:
		//motors[ID].torqueConst = B[2] & 0x7f;
		//motors[ID].torqueConst_readFlag = 0x00;
		pt();
		cout << 'm' << (int)ID << " Is_TrqCons=" << (B[2] & 0x7f) << endl;
		break;
	case Is_HighSpeed:
		//motors[ID].speedLimit = B[2] & 0x7f;
		//motors[ID].speedLimit_readFlag = 0x00;
		cout << 'm' << (int)ID << " Is_MainGain=" << (B[2] & 0x7f) << endl;
		break;
	case Is_HighAccel:
		//motors[ID].accelLimit = B[2] & 0x7f;
		//motors[ID].accelLimit_readFlag = 0x00;
		cout << 'm' << (int)ID << " Is_HighAccel=" << (B[2] & 0x7f) << endl;
		break;
	case Is_Drive_ID:
		//motors[ID].ID = B[2] & 0x7f;
		//motors[ID].ID_readFlag = 0x00;
		cout << 'm' << (int)ID << " Is_Drive_ID=" << (B[2] & 0x7f) << endl;
		break;
	case Is_PosOn_Range:
		motors[ID].posOnRange = B[2] & 0x7f;
		motors[ID].posOnRange_readFlag = 0x00;

		/* Output */
		pt();
		cout << 'm' << (int)ID << " Is_PosOn_Range=" << (B[2] & 0x7f) << endl;
		break;
	case Is_GearNumber:
		//motors[ID].gearN = ((B[2] & 0x007f) << 7) + (B[3] & 0x007f);
		//motors[ID].gearN_readFlag = 0x00;
		cout << 'm' << (int)ID << " Is_GearNumber=" << (((B[2] & 0x007f) << 7) + (B[3] & 0x007f)) << endl;
		break;
	case Is_Status:
		motors[ID].statusOnPosition = (B[2] & 0x7f) & 0x01;
		motors[ID].statusServoMode = ((B[2] & 0x7f) & 0x02) >> 1;
		motors[ID].statusAlarm = ((B[2] & 0x7f) & 0x1c) >> 2;
		motors[ID].statusMotion = ((B[2] & 0x7f) & 20) >> 5;
		motors[ID].statusPin2JP3 = ((B[2] & 0x7f) & 40) >> 6;
		motors[ID].status_readFlag = 0x00;

		pt();
		cout << 'm' << (int)ID << " Is_Status=" << (B[2] & 0x7f) << endl;
		pt();
		cout << 'm' << (int)ID << " OnPosition=" << (int)motors[ID].statusOnPosition << " ServoMode=" << (int)motors[ID].statusServoMode << " Alarm=" << (int)motors[ID].statusAlarm << " Motion=" << (int)motors[ID].statusMotion << " Pin2JP3=" << (int)motors[ID].statusPin2JP3 << endl;
		break;
	case Is_Config:

		//motors[ID].config_readFlag = 0x00;
		cout << 'm' << (int)ID << " Is_Config=" << (B[2] & 0x7f) << endl;
		break;
	case Is_AbsPos32:
		motors[ID].absPosition = calcSignedValue(packageLength, B);
		motors[ID].absPosition_readFlag = 0x00;
		pt();
		cout << 'm' << (int)ID << " Is_AbsPos32=" << DMM::calcSignedValue(packageLength, B) << endl;
		break;
	case Is_TrqCurrent:
		//motors[ID].torqueCurrent = calcSignedValue(packageLength, B);
		//motors[ID].torqueCurrent_readFlag = 0x00;
		pt();
		cout << 'm' << (int)ID << " Is_TrqCurrent=" << DMM::calcSignedValue(packageLength, B) << endl;
		break;
	default:;
	}
}

unsigned char DMM::calcCRC(unsigned char packageLength, unsigned char B[8])
{
	char CRC = 0;
	
	for (int i = 0; i < packageLength - 1; i++)	
	{
		CRC += B[i];
	}
	
	return CRC;
}

long DMM::calcSignedValue(unsigned char packageLength, unsigned char B[8])
{
	long lVal;
	lVal = (char)(B[2] << 1) >> 1; // Extends the 7 data bits of the third byte to 32bits, while including sign.
	
	for (int i = 3; i < packageLength - 1; i++)
	{
		lVal = (lVal << 7) + (B[i] & 0x7f);			// Shift 7 bits to the left and add the 7 data bits of the next byte.
	}

	return(lVal);
}

void DMM::setOrigin(unsigned char ID, serial::Serial &cSerial) {
	
	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_Origin;
	
	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B, cSerial);

}

void DMM::goAbsolutePos(unsigned char ID, int pos, serial::Serial &cSerial) {
	
	/* Define package parameters */
	unsigned char packageLength = 7;
	unsigned char functionCode = Go_Absolute_Pos;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(pos >> 21) & 0x0000007f;	// 1XXXXXXX Bit 22-28
	B[3] = NotStartBit | (unsigned char)(pos >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
	B[4] = NotStartBit | (unsigned char)(pos >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
	B[5] = NotStartBit | (unsigned char)pos & 0x0000007f;			// 1XXXXXXX Bit 1-7
	B[6] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B, cSerial);

	/* Output */
	pt();
	cout << 'm' << (int)ID << " goAbsolutePos=" << pos << endl;

}

void DMM::makeLinearLine(unsigned char ID, int pos) {
	
	/* Define package parameters */
	unsigned char packageLength = 7;
	unsigned char functionCode = Make_LinearLine;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(pos >> 21) & 0x0000007f;	// 1XXXXXXX Bit 22-28
	B[3] = NotStartBit | (unsigned char)(pos >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
	B[4] = NotStartBit | (unsigned char)(pos >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
	B[5] = NotStartBit | (unsigned char)pos & 0x0000007f;			// 1XXXXXXX Bit 1-7
	B[6] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);
}

void DMM::goRelativePos(unsigned char ID, int pos, serial::Serial &cSerial) {

	/* Define package parameters */
	unsigned char packageLength = 7;
	unsigned char functionCode = Go_Relative_Pos;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(pos >> 21) & 0x0000007f;	// 1XXXXXXX Bit 22-28
	B[3] = NotStartBit | (unsigned char)(pos >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
	B[4] = NotStartBit | (unsigned char)(pos >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
	B[5] = NotStartBit | (unsigned char)pos & 0x0000007f;			// 1XXXXXXX Bit 1-7
	B[6] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B, cSerial);

	/* Output */
	pt();
	cout << 'm' << (int)ID << " goRelativePos=" << pos << endl;
}

void DMM::makeCircularArc(unsigned char ID, int pos) {

	/* Define package parameters */
	unsigned char packageLength = 7;
	unsigned char functionCode = Make_CircularArc;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(pos >> 21) & 0x0000007f;	// 1XXXXXXX Bit 22-28
	B[3] = NotStartBit | (unsigned char)(pos >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
	B[4] = NotStartBit | (unsigned char)(pos >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
	B[5] = NotStartBit | (unsigned char)pos & 0x0000007f;			// 1XXXXXXX Bit 1-7
	B[6] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);
}

void DMM::assignDriveID(unsigned char ID, unsigned char id) {
	
	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Assign_Drive_ID;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | id;										// 1IIIIIII New drive ID
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readDriveID(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_Drive_ID;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::setDriveConfig(unsigned char ID, unsigned char cmdMode, unsigned char moveMode, unsigned char servoMode, unsigned char active) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_Drive_Config;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | ((cmdMode << 0) + (moveMode << 2) + (servoMode << 3) + (active << 5));		// 10DCCBAA Drive Config
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readDriveConfig(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_Drive_Config;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum
	
	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readDriveStatus(unsigned char ID, serial::Serial &cSerial) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_Drive_Status;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Update read flag */
	motors[ID].status_readFlag = 0x01;

	/* Send packet */
	sendPackage(packageLength, B, cSerial);

}

void DMM::turnConstSpeed(unsigned char ID, int speed) {
	
	/* Define package parameters */
	unsigned char packageLength = 6;
	unsigned char functionCode = Turn_ConstSpeed;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(speed >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
	B[3] = NotStartBit | (unsigned char)(speed >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
	B[4] = NotStartBit | (unsigned char)speed & 0x0000007f;			// 1XXXXXXX Bit 1-7
	B[5] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::squareWave(unsigned char ID, int amplitude) {

	/* Define package parameters */
	unsigned char packageLength = 6;
	unsigned char functionCode = Square_Wave;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(amplitude >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
	B[3] = NotStartBit | (unsigned char)(amplitude >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
	B[4] = NotStartBit | (unsigned char)amplitude & 0x0000007f;			// 1XXXXXXX Bit 1-7
	B[5] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::sinWave(unsigned char ID, int amplitude) {

	/* Define package parameters */
	unsigned char packageLength = 6;
	unsigned char functionCode = Sin_Wave;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(amplitude >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
	B[3] = NotStartBit | (unsigned char)(amplitude >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
	B[4] = NotStartBit | (unsigned char)amplitude & 0x0000007f;			// 1XXXXXXX Bit 1-7
	B[5] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::setFrequency(unsigned char ID, int frequency) {

	/* Define package parameters */
	unsigned char packageLength = 6;
	unsigned char functionCode = SS_Frequency;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(frequency >> 14) & 0x0000007f;	// 1XXXXXXX Bit 15-21
	B[3] = NotStartBit | (unsigned char)(frequency >> 7) & 0x0000007f;	// 1XXXXXXX Bit 8-14
	B[4] = NotStartBit | (unsigned char)frequency & 0x0000007f;			// 1XXXXXXX Bit 1-7
	B[5] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readPosition(unsigned char ID, serial::Serial &cSerial) {

	if (motors[ID].absPosition_readFlag == 0x00) {

		/* Define package parameters */
		unsigned char packageLength = 4;
		unsigned char functionCode = General_Read;

		/* Define package */
		unsigned char B[8];
		B[0] = StartBit & ID;											// 0IIIIIII Target motor id
		B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
		B[2] = NotStartBit | Is_AbsPos32;								// 10011011 Request position
		B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

		/* Send packet */
		sendPackage(packageLength, B, cSerial);

		motors[ID].absPosition_readFlag = 0x01;

		/* Output */
		pt();
		cout << 'm' << (int)ID << " readPosition" << endl;

	}
}

void DMM::readTorqueCurrent(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = General_Read;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | Is_TrqCurrent;								// 10011110 Request Torque Current
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

	/* Output */
	pt();
	cout << 'm' << (int)ID << " readPosition" << endl;
}

void DMM::setMainGain(unsigned char ID, unsigned char gain) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_MainGain;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | gain;										// 1XXXXXXX Gain
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::setSpeedGain(unsigned char ID, unsigned char gain) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_SpeedGain;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | gain;										// 1XXXXXXX Gain
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::setIntGain(unsigned char ID, unsigned char gain) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_IntGain;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | gain;										// 1XXXXXXX Gain
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::setTorqueCons(unsigned char ID, unsigned char constant) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_TrqCons;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | constant;									// 1XXXXXXX Gain
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::setSpeedLimit(unsigned char ID, unsigned char speed) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_HighSpeed;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | speed;										// 1XXXXXXX Speed
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::SetAccelLimit(unsigned char ID, unsigned char accel) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_HighAccel;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | accel;										// 1XXXXXXX Accel
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::setPosOnRange(unsigned char ID, unsigned char range) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Set_Pos_OnRange;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | range;										// 1XXXXXXX Range
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::setGearNumber(unsigned char ID, unsigned short gearN) {

	/* Define package parameters */
	unsigned char packageLength = 5;
	unsigned char functionCode = Set_GearNumber;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | (unsigned char)(gearN >> 7) & 0x007f;		// 1XXXXXXX Bit 8-14
	B[3] = NotStartBit | (unsigned char)gearN & 0x007f;				// 1XXXXXXX Bit 1-7
	B[4] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readMainGain(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_MainGain;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readSpeedGain(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_SpeedGain;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readIntGain(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_IntGain;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readTorqueCons(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_TrqCons;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

																	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readSpeedLimit(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_HighSpeed;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

																	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readAccelLimit(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_HighAccel;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::readPosOnRange(unsigned char ID, serial::Serial &cSerial) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_Pos_OnRange;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B, cSerial);

	/* Update read flag */
	motors[ID].posOnRange_readFlag = 0x01;

	/* Output */
	pt();
	cout << 'm' << (int)ID << " read_Pos_OnRange" << endl;
}

void DMM::readGearNumber(unsigned char ID) {

	/* Define package parameters */
	unsigned char packageLength = 4;
	unsigned char functionCode = Read_GearNumber;

	/* Define package */
	unsigned char B[8];
	B[0] = StartBit & ID;											// 0IIIIIII Target motor id
	B[1] = NotStartBit | (packageLength - 4) << 5 | functionCode;	// 1LLFFFFF Package length and function code
	B[2] = NotStartBit | 0;											// 10000000 Dummy
	B[3] = NotStartBit | calcCRC(packageLength, B);					// 1CCCCCCC CRC error checksum

	/* Send packet */
	sendPackage(packageLength, B);

}

void DMM::isMainGain()
{
}

void DMM::isSpeedGain()
{
}

void DMM::isIntGain()
{
}

void DMM::isTorqueCons()
{
}

void DMM::isSpeedLimit()
{
}

void DMM::isAccelLimit()
{
}

void DMM::isDriveID()
{
}

void DMM::isPosOnRange()
{
}

void DMM::isGearNumber()
{
}

void DMM::isStatus()
{
}

void DMM::isConfig()
{
}

void DMM::isAbsPosition()
{
}

void DMM::isTorqueCurrent()
{
}

void test(string port1, string port2) {
	
	long m = 30000;

	// Connect to Serial
	//serials.insert(std::make_pair(1, new serial::Serial(port2, 38400, serial::Timeout::simpleTimeout(1000))));
	serialPts[1] = new serial::Serial(port1, 38400, serial::Timeout::simpleTimeout(1000));
	serialPts[2] = new serial::Serial(port2, 38400, serial::Timeout::simpleTimeout(1000));
	cout << "Is the serial port open?";
	if (serialPts[1]->isOpen() && serialPts[2]->isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;


	pt();
	cout << "Testing started" << endl;
	//


	motors[1] = DMM::Motor(1, 1);
	motors[2] = DMM::Motor(2, 2);
	



	while (true) {

		// Instruct motor 1 to move back
		DMM::goAbsolutePos(1, m, *serialPts[1]);
		//Sleep(100);
		//DMM::readDriveStatus(1, *serialPts[1]);

		// Read positions of motors 1 and 2 until motor 1 is on range
		while (motors[1].absPosition_readFlag || abs(motors[1].absPosition - m) > 3) {
			//Sleep(10);
			DMM::readPosition(1, *serialPts[1]);
			//Sleep(10);
			DMM::readPackages(*serialPts[1]);
		}


		cout << endl << endl;
		// Instruct motor 1 to move back
		DMM::goAbsolutePos(1, -m, *serialPts[1]);
		//Sleep(100);
		//DMM::readDriveStatus(1, *serialPts[1]);

		// Read positions of motors 1 and 2 until motor 1 is on range
		while (motors[1].absPosition_readFlag || abs(motors[1].absPosition - (-m)) > 3) {
			//Sleep(10);
			DMM::readPosition(1, *serialPts[1]);
			//Sleep(10);
			DMM::readPackages(*serialPts[1]);
		}

		DMM::readPosOnRange(1, *serialPts[1]);
		DMM::readPackages(*serialPts[1]);

	}
}

void pt() {
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	cout << (now->tm_year + 1900) << '-'
		<< (now->tm_mon + 1) << '-'
		<< now->tm_mday << ' '
		<< now->tm_hour << ':'
		<< now->tm_min << ':'
		<< now->tm_sec << ' ';
}

using namespace std::chrono;
void ptm() {
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	cout << ms.count() << ' ';
}

int main() {
	test("COM4", "COM3");
}