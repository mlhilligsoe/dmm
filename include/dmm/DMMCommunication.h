#ifndef DMMCOMMUNICATION_H
#define DMMCOMMUNICATION_H

#include "serial\serial.h"

namespace DlgRun {
	void ReadPackage(serial::Serial&);
	void Get_Function(void);
	long Cal_SignValue(unsigned char One_Package[8]);
	void Send_Package(char ID, long Displacement, serial::Serial& cSerial);
	void Make_CRC_Send(unsigned char Plength, unsigned char B[8], serial::Serial& cSerial);
	void ReadMotorTorqueCurrent(serial::Serial& cSerial);
	void ReadMotorPosition32(serial::Serial& cSerial);
	void MoveMotorToAbsolutePosition32(char MotorID, long Pos32, serial::Serial& cSerial);
	void ReadMainGain(char MotorID, serial::Serial& cSerial);
}

#endif