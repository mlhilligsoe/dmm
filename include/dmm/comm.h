#ifndef DMM_COMM_H
#define DMM_COMM_H

#include <map>

#include "serial\serial.h"
#include "dmm\motor.h"

using std::map;

namespace DMM {

	void sendPackage(Motor& motor, unsigned char packageLength, unsigned char B[8]);
	void readPackages(serial::Serial& cSerial, map<unsigned char, Motor*> motors);
	void interpretPackage(Motor& motor, unsigned char packageLength, unsigned char B[8]);
	unsigned char calcCRC(unsigned char packageLength, unsigned char B[8]);
	long calcSignedValue(unsigned char packageLength, unsigned char B[8]);
	
	
	/* Drive input functions */
	void setOrigin(Motor& motor);
	void goAbsolutePos(Motor& motor, int pos);
	void goRelativePos(Motor& motor, int pos);
	void readDriveID(Motor& motor);
	void setDriveConfig(Motor& motor, unsigned char cmdMode, unsigned char moveMode, unsigned char servoMode, unsigned char active);
	void readDriveConfig(Motor& motor);
	void readDriveStatus(Motor& motor);
	void turnConstSpeed(Motor& motor, int speed);
	void readPosition(Motor& motor);
	void readTorqueCurrent(Motor& motor);
	void setMainGain(Motor& motor, unsigned char gain);
	void setSpeedGain(Motor& motor, unsigned char gain);
	void setIntGain(Motor& motor, unsigned char gain);
	void setTorqueCons(Motor& motor, unsigned char constant);
	void setSpeedLimit(Motor& motor, unsigned char speed);
	void setAccelLimit(Motor& motor, unsigned char accel);
	void setGearNumber(Motor& motor, unsigned short gearN);
	
	/* Drive output functions */
	void isDriveID(Motor& motor, unsigned char B[8]);
	void isStatus(Motor& motor, unsigned char B[8]);
	void isConfig(Motor& motor, unsigned char B[8]);
	void isAbsPosition(Motor& motor, unsigned char packageLength, unsigned char B[8]);
	void isTorqueCurrent(Motor& motor, unsigned char packageLength, unsigned char B[8]);
}


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

#endif

